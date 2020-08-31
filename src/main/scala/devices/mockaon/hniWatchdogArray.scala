package hni.blocks.devices.watchdog

import chisel3._
import Chisel.ImplicitConversions._
import chisel3.util._

import freechips.rocketchip.util.AsyncResetReg
import freechips.rocketchip.regmapper.{RegFieldDesc}
import sifive.blocks.util.{SlaveRegIF, GenericTimerIO}

class WatchdogArrayIO(Dogs:Int,Resets:Int,regWidth:Int,ncmp:Int,countWidth:Int,cmpWidth:Int ) extends Bundle{
  val mux = new SlaveRegIF(Resets)
  val pulsewidth = new SlaveRegIF(regWidth)
  val timerIO = new GenericTimerIO( regWidth,  ncmp,  4,  4,  countWidth, cmpWidth)
  override def cloneType = (new WatchdogArrayIO(Dogs,Resets,regWidth,ncmp,countWidth,cmpWidth)).asInstanceOf[this.type]
}

object WatchdogArray {
  def writeAnyExceptKey(regs: WatchdogArrayIO): Bool = {
    val tmp = WatchdogTimer.writeAnyExceptKey(regs.timerIO.regs , regs.timerIO.regs.key)
    (tmp || regs.mux.write.valid || regs.pulsewidth.write.valid)
  }
}

class WatchdogArray(Dogs: Int, Resets: Int, Ints: Int, Mode: hniWatchdogTimer.Modes, SafetySource: Int, SafetyEnable: Boolean,SourceWidth : Int, PulseWidth: Int = 32, Offset: Int = 0, regWidth: Int = 32, countWidth: Int=31, cmpWidth: Int=16) extends Module {

  require(SourceWidth > 0)
  require((Dogs >0) && (Dogs <= 32))
  require((Resets >=0) && (Resets <= 32))
  require((Ints >=0) && (Ints <= Dogs))
  require((PulseWidth >=0) && (PulseWidth <= 32))

  protected val ncmp = if(Mode == hniWatchdogTimer.timeout){1}else{2}  
  protected val prefix = "wdogarray"
  protected val prefix_dog = "wdog"
  
  val io = IO(new Bundle {
    val corerst = Input(Bool())
    val source = Input(UInt(SourceWidth.W))

    val inv = new SlaveRegIF(Resets)
    val key = new SlaveRegIF(32)
    val WDIO = Vec(Dogs,new WatchdogArrayIO(Dogs,Resets,regWidth,ncmp,countWidth,cmpWidth))

    val outputs = Output(Vec(Resets,Bool()))
    val interrupts = Output(Vec(Ints,Bool()))
  })

  // Register
  val inv_reg = RegEnable(io.inv.write.bits, io.inv.write.valid && unlocked)
  io.inv.read := inv_reg
  val mux_reg = for(i <- 0 until Dogs) yield {
    val reg = RegEnable(io.WDIO(i).mux.write.bits, io.WDIO(i).mux.write.valid && unlocked)
    io.WDIO(i).mux.read := reg
    reg
  }

  val pulsewidth_reg = for(i <- 0 until Dogs) yield {
    val reg = RegEnable(io.WDIO(i).pulsewidth.write.bits, io.WDIO(i).pulsewidth.write.valid && unlocked)
    io.WDIO(i).pulsewidth.read := reg
    reg
  }

  // Unlock
  protected lazy val unlocked = {
    val writeAnyseq = for(i<- 0 until Dogs) yield {
        WatchdogArray.writeAnyExceptKey(io.WDIO(i))
    }
    val writeAnyOr = (writeAnyseq.reduce(_||_) || io.inv.write.valid)
    val writeAny = if(SafetyEnable){writeAnyOr && (io.source == SafetySource) }else{writeAnyOr}
    AsyncResetReg(io.key.write.bits === WatchdogTimer.key && !writeAny, io.key.write.valid || writeAny)(0)
  }
  io.key.read := unlocked

  // Signals
  val mux_help = Wire(Vec(Resets,Vec(Dogs,Bool())))
  val PulseWidthCounter = RegInit(0.asTypeOf(Vec(Dogs,UInt(PulseWidth.W))))

  //val result: Nothing = pulsewidth_reg(0)

  // Add Watchdog Modules and connect signals
  val wdogs = for (i <- 0 until Dogs) yield
  {
      val wdog = Module(new WatchdogTimer(pcountWidth = countWidth, pcmpWidth = cmpWidth, pncmp = ncmp, pMode = Mode))
      if(i < Ints){
        io.interrupts(i) := wdog.io.ip(0)
      }
      wdog.io.corerst := io.corerst 
      wdog.io.unlocked := unlocked
      io.WDIO(i).timerIO.regs <> wdog.io.regs
      io.WDIO(i).timerIO.ip := wdog.io.ip
      //reg_ctrl.io.timerIO_wd(i).ip(0) := wdog.io.ip(0)
      //reg_ctrl.io.interrupts(i) := wdog.io.ip(0)
      wdog.io.output_reset := (PulseWidthCounter(i) >= pulsewidth_reg(i)) && pulsewidth_reg(i) > 0.U

      wdog
  }
  // Length of ResetPulse
  for (i <- 0 until Dogs){
    when(wdogs(i).io.rst){
      when(PulseWidthCounter(i) < pulsewidth_reg(i)){
        PulseWidthCounter(i) := PulseWidthCounter(i) + 1.U
      }
    }.otherwise{
      PulseWidthCounter(i) := 0.U 
    }
  }

  // Multiplex Watchdog to Outputs
  for(i <- 0 to Resets-1){
    for(x <- 0 to Dogs-1){
      mux_help(i)(x) := mux_reg(x)(i) && (wdogs(x).io.rst )
    }
    io.outputs(i) := mux_help(i).asUInt.orR ^ inv_reg(i)
  }



/*
  // We can't get this out of our WD Timers because they're Protected defs ...
  // Registers Per Watchdog
  protected def countLo_desc(dog:Int): RegFieldDesc =  if (countWidth > regWidth)  RegFieldDesc(s"${prefix_dog}${dog}_count", "Low bits of Counter", volatile=true) else RegFieldDesc(s"${prefix}${dog}_count", "Counter Register", volatile=true)
  protected def countHi_desc(dog:Int): RegFieldDesc = if (countWidth > regWidth) RegFieldDesc(s"${prefix_dog}${dog}_counthi", "High bits of Counter", volatile=true) else RegFieldDesc.reserved
  protected def s_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_s", "Scaled value of Counter", access=RegFieldAccessType.R, volatile=true)
  protected def cmp_desc(dog:Int): Seq[RegFieldDesc] = Seq.tabulate(ncmp){ i => RegFieldDesc(s"${prefix_dog}${dog}_cmp${i}", s"Comparator ${i}")}
  protected def feed_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_feed", "Feed register")
  //protected def cfg_desc(dog:Int): GenericTimerCfgDescs = DefaultGenericTimerCfgDescs(prefix_dog+dog.toString+"_", ncmp)
  protected def width_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_width", "PulseWidth Register", volatile=true)
  protected def mux_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_mux", "Mux Register", volatile=true)
  protected def cfg_desc(dog:Int): GenericTimerCfgDescs = DefaultGenericTimerCfgDescs(prefix_dog+dog.toString+"_", ncmp).copy(
    sticky = RegFieldDesc(s"${prefix_dog}${dog}_rsten", "Controls whether the comparator output can set the wdogrst bit and hence cause a full reset.",
      reset = Some(0)),
    deglitch = RegFieldDesc(s"${prefix_dog}${dog}_nOneshot", "Negated One Shot - When 0 WD does only one Reset Impulse"),
    running = RegFieldDesc(s"${prefix_dog}${dog}_wdogcoreawake", "Increment the watchdog counter if the processor is not asleep", reset=Some(0)),
    center = Seq.fill(ncmp){RegFieldDesc.reserved},
    extra = Seq.fill(ncmp){RegFieldDesc.reserved},
    gang = Seq.fill(ncmp){RegFieldDesc.reserved}
  )

*/
  // Registers Per Array
  protected def key_desc: RegFieldDesc = RegFieldDesc(s"${prefix}key", "Key Register")
  protected def inv_desc: RegFieldDesc = RegFieldDesc(s"${prefix}inv", "Inversion Register", volatile=true)
}

object mWatchdogArray extends App {
  chisel3.Driver.execute(Array("--target-dir", "generated/WatchdogArray"), () => new WatchdogArray( Dogs=3, Resets=2, Ints=1, Mode= hniWatchdogTimer.both, SafetySource = 5, SafetyEnable= true,SourceWidth =4))
}