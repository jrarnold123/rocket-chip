// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.tilelink

import Chisel._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.util._

object ClientStates {
  val width = 5
  val M = UInt(0, width)
  val Mr = UInt(1, width)
  val Mrr = UInt(2, width)
  val E = UInt(3, width)
  val Er = UInt(4, width)
  val Err = UInt(5, width)
  val SMg = UInt(6, width)
  val SMgr = UInt(7, width)
  val SMgrr = UInt(8, width)
  val S = UInt(9, width)
  val Sr = UInt(10, width)
  val ISrr = UInt(11, width)
  val ISgd = UInt(12, width)
  val ISgdr = UInt(13, width)
  val ISgdrr = UInt(14, width)
  val IMg = UInt(15, width)
  val IMgr = UInt(16, width)
  val IMgrr = UInt(17, width)
  val I = UInt(18, width)
  val Ir = UInt(19, width)
  val IIrr = UInt(20, width)

  def hasReadPermission(state: UInt): Bool = state < ISgd
  def hasWritePermission(state: UInt): Bool = state < SMg
}

object MemoryOpCategories extends MemoryOpConstants {
  def wr = Cat(Bool(true), Bool(true))   // Op actually writes
  def wi = Cat(Bool(false), Bool(true))  // Future op will write
  def rd = Cat(Bool(false), Bool(false)) // Op only reads

  def categorize(cmd: UInt): UInt = {
    val cat = Cat(isWrite(cmd), isWriteIntent(cmd))
    //assert(cat.isOneOf(wr,wi,rd), "Could not categorize command.")
    cat
  }
}

/** Stores the client-side coherence information,
  * such as permissions on the data and whether the data is dirty.
  * Its API can be used to make TileLink messages in response to
  * memory operations, cache control oeprations, or Probe messages.
  */
class ClientMetadata extends Bundle {
  /** Actual state information stored in this bundle */
  val state = UInt(width = ClientStates.width)

  /** Metadata equality */
  def ===(rhs: UInt): Bool = state === rhs
  def ===(rhs: ClientMetadata): Bool = state === rhs.state
  def =/=(rhs: ClientMetadata): Bool = !this.===(rhs)

  /** Is the block's data present in this cache */
  def isValid(dummy: Int = 0): Bool = state < ISrr

  

/** Factories for ClientMetadata, including on reset */
object ClientMetadata {
  def apply(perm: UInt) = {
    val meta = Wire(new ClientMetadata)
    meta.state := perm
    meta
  }
  def onReset = ClientMetadata(ClientStates.I)
  def maximum = ClientMetadata(ClientStates.M)
}
