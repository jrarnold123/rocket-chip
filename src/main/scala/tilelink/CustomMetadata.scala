// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.tilelink

import Chisel._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.util._

object CustomClientStates {
  val width = 2
  def I = UInt(0, width)
  def S  = UInt(1, width)
  def E   = UInt(2, width)
  def M   = UInt(3, width)

  def hasReadPermission(state: UInt): Bool = state > I
  def hasWritePermission(state: UInt): Bool = (state === CustomClientStates.E || state === CustomClientStates.M)
}

object CustomMemoryOpCategories extends MemoryOpConstants {
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
class CustomClientMetadata extends Bundle {
  /** Actual state information stored in this bundle */
  val state = UInt(width = CustomClientStates.width)

  /** Metadata equality */
  def ===(rhs: UInt): Bool = state === rhs
  def ===(rhs: CustomClientMetadata): Bool = state === rhs.state
  def =/=(rhs: CustomClientMetadata): Bool = !this.===(rhs)

  /** Is the block's data present in this cache */
  def isValid(dummy: Int = 0): Bool = state > CustomClientStates.I

  def isStable(): Bool = (state === CustomClientStates.M || state === CustomClientStates.E || state === CustomClientStates.S || state === CustomClientStates.I)

  def isDirty(): Bool = state >= CustomClientStates.M

  def hasReadPermission(): Bool = state > CustomClientStates.I

  def hasWritePermission(): Bool = (state === CustomClientStates.E || state === CustomClientStates.M)

  //def hasData(): Bool = state >=

  /** Determine whether this cmd misses, and the new state (on hit) or param to be sent (on miss) */
  private def growStarter(cmd: UInt): (Bool, UInt) = {
    import CustomMemoryOpCategories._
    import TLPermissions._
    import CustomClientStates._
    val c = categorize(cmd)
    MuxTLookup(Cat(c, state), (Bool(false), UInt(0)), Seq(
        //(effect, am now) -> (was a hit,   next)
        Cat(rd, M)   -> (Bool(true),  M),
        Cat(rd, E)   -> (Bool(true),  E),
        Cat(rd, S)  -> (Bool(true),  S),
        Cat(wi, M)   -> (Bool(true),  M),
        Cat(wi, E)   -> (Bool(true),  E),
        Cat(wr, M)   -> (Bool(true),  M),
        Cat(wr, E)   -> (Bool(true),  M),
        //(effect, am now) -> (was a miss,  param)
        Cat(rd, I) -> (Bool(false), NtoB),
        Cat(wi, S)  -> (Bool(false), BtoT),
        Cat(wi, I) -> (Bool(false), NtoT),
        Cat(wr, S)  -> (Bool(false), BtoT),
        Cat(wr, I) -> (Bool(false), NtoT)))
  }

  /** Determine what state to go to after miss based on Grant param
    * For now, doesn't depend on state (which may have been Probed).
    */
  private def growFinisher(cmd: UInt, param: UInt): UInt = {
    import CustomMemoryOpCategories._
    import TLPermissions._
    import CustomClientStates._
    val c = categorize(cmd)
    //assert(c === rd || param === toT, "Client was expecting trunk permissions.")

    MuxLookup(Cat(c, param), I, Seq(
    //(effect param) -> (next)
      Cat(rd, toB)   -> S,
      Cat(rd, toT)   -> E,
      Cat(wi, toT)   -> E,
      Cat(wr, toT)   -> M)).asUInt
  }
  private def growFinisher(param: UInt): UInt = {
    import TLPermissions._
    import CustomClientStates._
    MuxLookup(param, I, Seq(
    //(effect param) -> (next)
      toB   -> S,
      toT   -> E)).asUInt
  }

  /** Does this cache have permissions on this block sufficient to perform op,
    * and what to do next (Acquire message param or updated metadata). */
  def onAccess(cmd: UInt): (Bool, UInt, CustomClientMetadata) = {
    val r = growStarter(cmd)
    (r._1, r._2, CustomClientMetadata(r._2))
  }

  def onWrite(cmd: UInt): CustomClientMetadata = {
    import CustomMemoryOpCategories._
    assert(isWrite(cmd), "PLEASE")
    val temp = CustomClientMetadata(CustomClientStates.I)
    val temp2 = Wire(UInt(0))
    when (state === CustomClientStates.E) {
      temp := CustomClientMetadata(CustomClientStates.M)
    } .otherwise {
      temp := CustomClientMetadata(state)
    }
    temp
  }

  /** Does a secondary miss on the block require another Acquire message */
  def onSecondaryAccess(first_cmd: UInt, second_cmd: UInt): (Bool, Bool, UInt, CustomClientMetadata, UInt) = {
    import CustomMemoryOpCategories._
    val r1 = growStarter(first_cmd)
    val r2 = growStarter(second_cmd)
    val needs_second_acq = isWriteIntent(second_cmd) && !isWriteIntent(first_cmd)
    val hit_again = r1._1 && r2._1
    val dirties = categorize(second_cmd) === wr
    val biggest_grow_param = Mux(dirties, r2._2, r1._2)
    val dirtiest_state = CustomClientMetadata(biggest_grow_param)
    val dirtiest_cmd = Mux(dirties, second_cmd, first_cmd)
    (needs_second_acq, hit_again, biggest_grow_param, dirtiest_state, dirtiest_cmd)
  }

  /** Metadata change on a returned Grant */
  def onGrant(cmd: UInt, param: UInt): CustomClientMetadata = CustomClientMetadata(growFinisher(cmd, param))
  def onGrant(param: UInt): CustomClientMetadata = CustomClientMetadata(growFinisher(param))

  /** Determine what state to go to based on Probe param */
  private def shrinkHelper(param: UInt): (Bool, UInt, UInt) = {
    import CustomClientStates._
    import TLPermissions._
    MuxTLookup(Cat(param, state), (Bool(false), UInt(0), UInt(0)), Seq(
    //(wanted, am now)  -> (hasDirtyData, resp, next)
      Cat(toT, M)   -> (Bool(true),  TtoT, E),
      Cat(toT, E)   -> (Bool(false), TtoT, E),
      Cat(toT, S)  -> (Bool(false), BtoB, S),
      Cat(toT, I) -> (Bool(false), NtoN, I),
      Cat(toB, M)   -> (Bool(true),  TtoB, S),
      Cat(toB, E)   -> (Bool(false), TtoB, S),  // Policy: Don't notify on clean downgrade
      Cat(toB, S)  -> (Bool(false), BtoB, S),
      Cat(toB, I) -> (Bool(false), NtoN, I),
      Cat(toN, M)   -> (Bool(true),  TtoN, I),
      Cat(toN, E)   -> (Bool(false), TtoN, I), // Policy: Don't notify on clean downgrade
      Cat(toN, S)  -> (Bool(false), BtoN, I), // Policy: Don't notify on clean downgrade
      Cat(toN, I) -> (Bool(false), NtoN, I)))
  }

  /** Translate cache control cmds into Probe param */
  private def cmdToPermCap(cmd: UInt): UInt = {
    import CustomMemoryOpCategories._
    import TLPermissions._
    MuxLookup(cmd, toN, Seq(
      M_FLUSH   -> toN,
      M_PRODUCE -> toB,
      M_CLEAN   -> toT))
  }

  def onCacheControl(cmd: UInt): (Bool, UInt, CustomClientMetadata) = {
    val r = shrinkHelper(cmdToPermCap(cmd))
    (r._1, r._2, CustomClientMetadata(r._3))
  }

  def onProbe(param: UInt): (Bool, UInt, CustomClientMetadata) = { 
    val r = shrinkHelper(param)
    (r._1, r._2, CustomClientMetadata(r._3))
  }

}
  

/** Factories for ClientMetadata, including on reset */
object CustomClientMetadata {
  def apply(perm: UInt) = {
    val meta = Wire(new CustomClientMetadata)
    meta.state := perm
    meta
  }
  def onReset = CustomClientMetadata(CustomClientStates.I)
  def maximum = CustomClientMetadata(CustomClientStates.M)
}
