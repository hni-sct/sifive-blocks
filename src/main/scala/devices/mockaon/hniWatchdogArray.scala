package hni.blocks.devices.watchdog

import chisel3._
import Chisel.ImplicitConversions._
import chisel3.util._

import freechips.rocketchip.util.AsyncResetReg
import freechips.rocketchip.regmapper._
import sifive.blocks.util.{SlaveRegIF, GenericTimerIO, GenericTimer}

class WatchdogArrayIO(Dogs:Int,Resets:Int,regWidth:Int,ncmp:Int,countWidth:Int,cmpWidth:Int ) extends Bundle{
  val mux = new SlaveRegIF(Resets)
  val pulsewidth = new SlaveRegIF(regWidth)
  val timerIO = new GenericTimerIO( regWidth,  ncmp,  4,  4,  countWidth, cmpWidth)
  override def cloneType = (new WatchdogArrayIO(Dogs,Resets,regWidth,ncmp,countWidth,cmpWidth)).asInstanceOf[this.type]
}

class WatchdogArray(Dogs: Int, Resets: Int, Ints: Int, Mode: hniWatchdogTimer.Modes, PulseWidth: Int = 32, Offset: Int = 0, regWidth: Int = 32, countWidth: Int=31, cmpWidth: Int=16) extends Module {

  require((Dogs >0) && (Dogs <= 32))
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
    val writeAny = (writeAnyseq.reduce(_||_) || io.inv.write.valid)
    AsyncResetReg(io.key.write.bits === WatchdogTimer.key && !writeAny, io.key.write.valid || writeAny)(0)
  }
  io.key.read := unlocked

  // Signals
  val mux_help = Wire(Vec(Resets,Vec(Dogs,Bool())))
  val PulseWidthCounter = RegInit(0.asTypeOf(Vec(Dogs,UInt(PulseWidth.W))))

  //val result: Nothing = pulsewidth_reg(0)
  //val unlock = if (SafetyEnable){unlocked && (io.source === SafetySource)}else{unlocked}
  // Add Watchdog Modules and connect signals
  val wdogs = for (i <- 0 until Dogs) yield
  {
      val wdog = Module(new WatchdogTimer(pcountWidth = countWidth, pcmpWidth = cmpWidth, pncmp = ncmp, pMode = Mode, pprefix = s"${prefix_dog}${i}"))
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
  for(i <- 0 to Resets-1){
    for(x <- 0 to Dogs-1){
      mux_help(i)(x) := mux_reg(x)(i) && (wdogs(x).io.rst )
    }
    io.outputs(i) := mux_help(i).asUInt.orR ^ inv_reg(i)
  }

  // Register Descriptions
  protected def wdogs_seq : Seq[Seq[(Int, Seq[RegField])]] = Seq.tabulate( Dogs )(i => GenericTimer.timerRegMap(wdogs(i), Offset + (2*regWidth/8) + (WatchdogArray.DogWidth*i), regWidth/8))
  protected def width_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_width", "PulseWidth Register", volatile=true)
  protected def mux_desc(dog:Int): RegFieldDesc = RegFieldDesc(s"${prefix_dog}${dog}_mux", "Mux Register", volatile=true)
  protected def key_desc: RegFieldDesc = RegFieldDesc(s"${prefix}_key", "Key Register")
  protected def inv_desc: RegFieldDesc = RegFieldDesc(s"${prefix}_inv", "Inversion Register", volatile=true)
}

object WatchdogArray {
  val DogWidth = 44

  def writeAnyExceptKey(regs: WatchdogArrayIO): Bool = {
    val tmp = WatchdogTimer.writeAnyExceptKey(regs.timerIO.regs , regs.timerIO.regs.key)
    (tmp || regs.mux.write.valid || regs.pulsewidth.write.valid)
  }

  def wdRegMap(arrIO:WatchdogArrayIO,arr: WatchdogArray, offset: Int, regBytes: Int, dog: Int): Seq[(Int, Seq[RegField])] = {
    val pulsewidth = Seq( offset + (DogWidth-regBytes) -> Seq(arrIO.pulsewidth.toRegField(Some(arr.width_desc(dog)))) )
    val mux = Seq( offset + DogWidth -> Seq(arrIO.mux.toRegField(Some(arr.mux_desc(dog)))) )
    pulsewidth ++ mux
  }

  // Seq( (i1, reg1), (i2,reg2) )  -> This sequence holds elements, with i1 offset of reg1 etc. ++ concats inner elements
  def arrRegMap(arr: WatchdogArray, offset: Int, regBytes: Int, Dogs : Int): Seq[(Int, Seq[RegField])] = {
    val inv = Seq( offset+4 -> Seq(arr.io.inv.toRegField(Some(arr.inv_desc))) )
    val key = Seq( offset -> Seq(arr.io.key.toRegField(Some(arr.key_desc))) )
    val wdogs = Seq.tabulate( Dogs )(i => WatchdogArray.wdRegMap(arr.io.WDIO(i),arr, offset+8+(DogWidth*i), regBytes, i)) 
    key ++ inv ++ arr.wdogs_seq.flatten ++ wdogs.flatten  
  }
}


object mWatchdogArray extends App {
  chisel3.Driver.execute(Array("--target-dir", "generated/WatchdogArray"), () => new WatchdogArray( Dogs=3, Resets=2, Ints=1, Mode= hniWatchdogTimer.both ))
}