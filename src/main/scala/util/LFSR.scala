// See LICENSE for license details.
package hni.blocks.util
import chisel3._
import Chisel.ImplicitConversions._
import chisel3.util._

class FibonacciLFSR(
  width: Int,
  taps: Set[Int],
  init: Int = 0
  ) extends Module {
      require(taps.size>0)
  val io = IO(new Bundle{
      val out = Output(UInt(width.W))
      val increment = Input(Bool())
  }
  )

    val taps_seq = taps.toSeq
    val in = if(taps_seq.length > 1){
                Seq.tabulate(taps_seq.length)(i => io.out( taps_seq(i) )).reduce(_^_)
            }else{
                io.out(taps_seq(0))
            }

    val reg = Seq.tabulate(width) (i => RegInit((init.U(32.W))(i)) ) 
    for(i<- 0 until width){
        if (i==0){
            when(io.increment){
                reg(i) := in
            }
        }else{
            when(io.increment){
                reg(i) := reg(i-1)
            }
        }
    }
    
    io.out := VecInit(reg).asUInt()
}
// java -jar rocket-chip/sbt-launch.jar ++2.12.4 "runMain hni.blocks.util.mFibonacci"
object mFibonacci extends App {
  chisel3.Driver.execute(Array("--target-dir", "generated/LFSR"), () => new FibonacciLFSR( width=32, taps = Set(4, 3),0x55555 ))
}