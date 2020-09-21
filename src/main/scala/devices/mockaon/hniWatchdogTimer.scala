// See LICENSE for license details.
package hni.blocks.devices.watchdog

import Chisel._
import Chisel.ImplicitConversions._
import chisel3.experimental.MultiIOModule
import freechips.rocketchip.util.AsyncResetReg
import freechips.rocketchip.regmapper.{RegFieldDesc}

import sifive.blocks.util.{SlaveRegIF, GenericTimer, GenericTimerIO, GenericTimerCfgRegIFC, DefaultGenericTimerCfgDescs}

object hniWatchdogTimer extends Enumeration {
  type Modes = Value
  val timeout, window, both = Value
}

object WatchdogTimer {
  def writeAnyExceptKey(regs: Bundle, keyReg: SlaveRegIF): Bool = {
    regs.elements.values.filter(_ ne keyReg).map({
      case c: GenericTimerCfgRegIFC => c.anyWriteValid
      case v: Vec[SlaveRegIF] @unchecked => v.map(_.write.valid).reduce(_||_)
      case s: SlaveRegIF => s.write.valid
    }).reduce(_||_)
  }

  val key = 0x51F15E
}

class WatchdogTimer(pcountWidth:Int=31,pcmpWidth:Int=16,pncmp:Int=1, pMode : hniWatchdogTimer.Modes = hniWatchdogTimer.timeout, pprefix : String = "wdog") extends MultiIOModule with GenericTimer {
  def prefix = pprefix
  protected def countWidth = pcountWidth//31
  protected def cmpWidth = pcmpWidth//16
  protected def ncmp = pncmp//1
  protected lazy val countAlways = AsyncResetReg(io.regs.cfg.write.countAlways, io.regs.cfg.write_countAlways && unlocked)(0)
  override protected lazy val countAwake = AsyncResetReg(io.regs.cfg.write.running, io.regs.cfg.write_running && unlocked)(0)
  protected lazy val countEn = {
    val corerstSynchronized = Reg(next = Reg(next = io.corerst))
    (countAlways || (countAwake && !corerstSynchronized))&&(~io.rst)
  }
  override protected lazy val rsten = AsyncResetReg(io.regs.cfg.write.sticky, io.regs.cfg.write_sticky && unlocked)(0)
  protected lazy val elapsed_ip = if(pMode == hniWatchdogTimer.timeout){elapsed(0)}else{elapsed(0) || elapsed(1)}
  protected lazy val ip = RegEnable(Vec(Seq(io.regs.cfg.write.ip(0) || elapsed_ip)), (io.regs.cfg.write_ip(0) && unlocked) || elapsed_ip) 
  override protected lazy val unlocked = io.unlocked
  protected lazy val feed = {
    val food = 0xD09F00D
    unlocked && io.regs.feed.write.valid && io.regs.feed.write.bits === food
  }

  override protected lazy val deglitch = if(pMode == hniWatchdogTimer.both){ RegEnable(io.regs.cfg.write.deglitch, io.regs.cfg.write_deglitch && unlocked) }else{ Bool(false) }

  override protected lazy val elapsed = Vec.tabulate(ncmp){i => 
    i match{
      case 0 => if (pMode == hniWatchdogTimer.timeout){
                  s >= cmp(i)
                }else if (pMode == hniWatchdogTimer.window){
                  (s <= cmp(i)) && feed && (s =/= 0.U)
                }else{
                  Mux(deglitch, (s <= cmp(i)) && feed && (s =/= 0.U), s >= cmp(i))
                }
                  
      case _ => if(pMode==hniWatchdogTimer.both){
                  (s >= cmp(i)) && deglitch
                }else{
                  s >= cmp(i)
                }
    }  
  }
  override protected lazy val countReset = feed || (zerocmp && elapsed.asUInt().orR())

  // The Scala Type-Chekcher seems to have a bug and I get a null pointer during the Scala compilation
  // if I don't do this temporary assignment.
  val tmpStickyDesc =  RegFieldDesc("wdogrsten", "Controls whether the comparator output can set the wdogrst bit and hence cause a full reset.",
      reset = Some(0))
  
  val tmpdeglitch = if (pMode == hniWatchdogTimer.both){
                  RegFieldDesc("wdogmode", "Sets the Watchdog Mode 0 = Timeout, 1 = Window", reset=Some(0))
                }else{
                  RegFieldDesc.reserved
                }

  override protected lazy val cfg_desc = DefaultGenericTimerCfgDescs("wdog", ncmp).copy(
    sticky = tmpStickyDesc,
    deglitch = tmpdeglitch,
    running = RegFieldDesc("wdogcoreawake", "Increment the watchdog counter if the processor is not asleep", reset=Some(0)),
    center = Seq.fill(ncmp){RegFieldDesc.reserved},
    extra = Seq.fill(ncmp){RegFieldDesc.reserved},
    gang = Seq.fill(ncmp){RegFieldDesc.reserved}

  )

  lazy val io = IO(new GenericTimerIO(regWidth, ncmp, maxcmp, scaleWidth, countWidth, cmpWidth) {
    val corerst = Bool(INPUT)
    val unlocked = Bool(INPUT)
    val output_reset = Bool(INPUT)
    val rst = Bool(OUTPUT)
  }
  )

  val output_reset = RegInit(false.B)
  output_reset := rsten && elapsed.asUInt().orR()
  io.rst := AsyncResetReg(Bool(true), output_reset.asClock(), reset || io.output_reset)
}
//java -jar rocket-chip/sbt-launch.jar ++2.12.4 "runMain hni.blocks.devices.watchdog.mWatchdog"
object mWatchdog extends App {
  chisel3.Driver.execute(Array("--target-dir", "generated/Watchdog"), () => new WatchdogTimer(pncmp=2, pMode=hniWatchdogTimer.both))
}