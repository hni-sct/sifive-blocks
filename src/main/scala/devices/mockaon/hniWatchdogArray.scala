package hni.blocks.devices.watchdog
import hni.blocks.util._

import chisel3._
import Chisel.ImplicitConversions._
import chisel3.util._

import freechips.rocketchip.util.AsyncResetReg
import freechips.rocketchip.regmapper._
import sifive.blocks.util.{SlaveRegIF, GenericTimerIO, GenericTimer, GenericTimerCfgDescs, DefaultGenericTimerCfgDescs}

class WatchdogArrayIO(Dogs:Int,Resets:Int,regWidth:Int,ncmp:Int,countWidth:Int,cmpWidth:Int, PulseWidth: Int ) extends Bundle{
  val mux = new SlaveRegIF(Resets)
  val pulsewidth = new SlaveRegIF(PulseWidth)
  val timerIO = new GenericTimerIO( regWidth,  ncmp,  4,  4,  countWidth, cmpWidth)
  override def cloneType = (new WatchdogArrayIO(Dogs,Resets,regWidth,ncmp,countWidth,cmpWidth,PulseWidth)).asInstanceOf[this.type]
}

class WatchdogArray(Dogs: Int, Resets: Int, Ints: Int, Mode: hniWatchdogTimer.Modes, PulseWidth: Int = 32, Offset: Int = 0, 
                    PRBS : Boolean = false, PRBS_Set : Set[Int] = Set(31,28), Key :Int = 0x51F15E, 
                    regWidth: Int = 32, countWidth: Int=31, cmpWidth: Int=16) extends Module {

  require((Resets >=0) && (Resets <= 32))
  require((Ints >=0) && (Ints <= Dogs))
  require((PulseWidth >=0) && (PulseWidth <= 32))

  protected val ncmp = if(Mode == hniWatchdogTimer.timeout){1}else{2}  
  protected val prefix = "wdogarray"
  protected val prefix_dog = "wdog"
  
  val io = IO(new Bundle {
    val corerst = Input(Bool())

    val inv = new SlaveRegIF(Resets)
    val key = new SlaveRegIF(32)
    val WDIO = Vec(Dogs,new WatchdogArrayIO(Dogs,Resets,regWidth,ncmp,countWidth,cmpWidth,PulseWidth))

    val outputs = Output(Vec(Resets,Bool()))
    val interrupts = Output(Vec(Ints,Bool()))
  })

  // Unlock
  protected lazy val unlocked : Bool = {
    val writeAnyseq = for(i<- 0 until Dogs) yield {
        WatchdogArray.writeAnyExceptKey(io.WDIO(i))
    }
    val writeAny = (writeAnyseq.reduce(_||_) || io.inv.write.valid)
    if(!PRBS){
      AsyncResetReg(io.key.write.bits === Key && !writeAny, io.key.write.valid || writeAny)(0)
    }else{
      val key_lfsr = Module(new FibonacciLFSR(32, PRBS_Set,Key))
      val out =  AsyncResetReg(io.key.write.bits === key_lfsr.io.out && !writeAny, io.key.write.valid || writeAny)(0)
      //val unlocked_ff = RegNext(out)
      key_lfsr.io.increment := writeAny && out
      out      
    }
  }
  io.key.read := unlocked

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

  // Signals
  val mux_help = Wire(Vec(Resets,Vec(Dogs,Bool())))
  val PulseWidthCounter = RegInit(0.asTypeOf(Vec(Dogs,UInt(PulseWidth.W))))

  // Add Watchdog Modules and connect signals
  val wdogs = for (i <- 0 until Dogs) yield
  {
      val wdog = Module(new WatchdogTimer(pcountWidth = countWidth, pcmpWidth = cmpWidth,  pMode = Mode))
      if(i < Ints){
        io.interrupts(i) := wdog.io.ip(0)
      }
      wdog.io.corerst := io.corerst 
      wdog.io.unlocked := unlocked
      io.WDIO(i).timerIO.regs <> wdog.io.regs
      io.WDIO(i).timerIO.ip := wdog.io.ip
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
  for(i <- 0 until Resets){
    for(x <- 0 until Dogs){
      mux_help(i)(x) := mux_reg(x)(i) && (wdogs(x).io.rst )
    }
    io.outputs(i) := mux_help(i).asUInt.orR ^ inv_reg(i)
  }

  // Register Descriptions
  // We can't get these Registers out of the Watchdogs because they're Protected ...
  protected def countLo_desc(dog:Int): RegFieldDesc =  if (countWidth > regWidth)  RegFieldDesc(s"${prefix_dog}${dog}_count", "Low bits of Counter", volatile=true) else RegFieldDesc(s"${prefix}${dog}_count", "Counter Register", volatile=true)
  protected def countHi_desc(dog:Int): RegFieldDesc = if (countWidth > regWidth) RegFieldDesc(s"${prefix_dog}${dog}_counthi", "High bits of Counter", volatile=true) else RegFieldDesc.reserved
  protected def s_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_s", "Scaled value of Counter", access=RegFieldAccessType.R, volatile=true)
  protected def cmp_desc(dog:Int): Seq[RegFieldDesc] = Seq.tabulate(ncmp){ i => RegFieldDesc(s"${prefix_dog}${dog}_cmp${i}", s"Comparator ${i}")}
  protected def feed_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_feed", "Feed register")
  protected def cfg_desc(dog:Int): GenericTimerCfgDescs = DefaultGenericTimerCfgDescs(prefix_dog+dog.toString+"_", ncmp).copy(
    sticky = RegFieldDesc(s"${prefix_dog}${dog}_rsten", "Controls whether the comparator output can set the wdogrst bit and hence cause a full reset.",
      reset = Some(0)),
    deglitch = RegFieldDesc(s"${prefix_dog}${dog}_nMode", "Sets the Watchdog Mode 0 = Timeout, 1 = Window"),
    running = RegFieldDesc(s"${prefix_dog}${dog}_wdogcoreawake", "Increment the watchdog counter if the processor is not asleep", reset=Some(0)),
    center = Seq.fill(ncmp){RegFieldDesc.reserved},
    extra = Seq.fill(ncmp){RegFieldDesc.reserved},
    gang = Seq.fill(ncmp){RegFieldDesc.reserved}
  )

  protected def pulsewidth_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_pulsewidth", "PulseWidth Register")
  protected def mux_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_mux", "Mux Register")
  protected def key_desc: RegFieldDesc = RegFieldDesc(s"${prefix}_key", "Key Register")
  protected def inv_desc: RegFieldDesc = RegFieldDesc(s"${prefix}_inv", "Inversion Register")
}

object WatchdogArray {
  val DogWidth = 48

  def writeAnyExceptKey(regs: WatchdogArrayIO): Bool = {
    val tmp = WatchdogTimer.writeAnyExceptKey(regs.timerIO.regs , regs.timerIO.regs.key)
    (tmp || regs.mux.write.valid || regs.pulsewidth.write.valid)
  }

  
  def wdRegMap(arr: WatchdogArray, offset: Int, regBytes: Int, dog: Int): Seq[(Int, Seq[RegField])] = {
    val cfgRegs = Seq(offset -> arr.io.WDIO(dog).timerIO.regs.cfg.toRegFields(s"${arr.prefix_dog}${dog}", arr.cfg_desc(dog)))
    val regs = Seq(
        2 -> (arr.io.WDIO(dog).timerIO.regs.countLo, arr.countLo_desc(dog)),
        3 -> (arr.io.WDIO(dog).timerIO.regs.countHi, arr.countHi_desc(dog)),
        4 -> (arr.io.WDIO(dog).timerIO.regs.s, arr.s_desc(dog)),
        6 -> (arr.io.WDIO(dog).timerIO.regs.feed, arr.feed_desc(dog)),
    )
    val cmpRegs = (arr.io.WDIO(dog).timerIO.regs.cmp zip arr.cmp_desc(dog)).zipWithIndex map { case ((r, d), i) => (8 + i) -> (r, d) }
    val otherRegs = for ((i, (r, d)) <- (regs ++ cmpRegs)) yield (offset + regBytes*i) -> Seq(r.toRegField(Some(d)))

    val pulsewidth = Seq( offset + (DogWidth-2*regBytes) -> Seq(arr.io.WDIO(dog).pulsewidth.toRegField(Some(arr.pulsewidth_desc(dog)))) )
    val mux = Seq( offset + (DogWidth-regBytes) -> Seq(arr.io.WDIO(dog).mux.toRegField(Some(arr.mux_desc(dog)))) )
    cfgRegs ++ otherRegs ++ pulsewidth ++ mux
  }

  // Seq( (i1, reg1), (i2,reg2) )  -> This sequence holds elements, with i1 offset of reg1 etc. ++ concats inner elements
  def arrRegMap(arr: WatchdogArray, offset: Int, regBytes: Int, Dogs : Int): Seq[(Int, Seq[RegField])] = {
    val inv = Seq( offset+4 -> Seq(arr.io.inv.toRegField(Some(arr.inv_desc))) )
    val key = Seq( offset -> Seq(arr.io.key.toRegField(Some(arr.key_desc))) )
    val wdogs = Seq.tabulate( Dogs )(i => WatchdogArray.wdRegMap(arr, offset+8+(DogWidth*i), regBytes, i)) 
    key ++ inv ++ wdogs.flatten  
  }
}

//java -jar rocket-chip/sbt-launch.jar ++2.12.4 "runMain hni.blocks.devices.watchdog.mWatchdogArray"
//verilator -Wall --trace -cc WatchdogArray.v
//make -f VWatchdogArray.mk
//g++ -I obj_dir -I/usr/share/verilator/include VWatchdogArray.cpp VWatchdogArray__Trace.cpp VWatchdogArray__Trace__Slow.cpp VWatchdogArray__Syms.cpp usr/watchdogreg.cpp usr/testbench.cpp /usr/share/verilator/include/verilated.cpp /usr/share/verilator/include/verilated_vcd_c.cpp -o usr/testbench.o 
object mWatchdogArray extends App {
  chisel3.Driver.execute(Array("--target-dir", "generated/WatchdogArray"), () => new WatchdogArray( Dogs=9, Resets=2, Ints=1, Mode= hniWatchdogTimer.both, PulseWidth=32, Offset = 0, PRBS = true, PRBS_Set = Set(6,7,30), Key = 0x51F15E, regWidth = 32, countWidth = 31, cmpWidth =16  ))
}