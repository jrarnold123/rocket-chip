// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.tilelink

import Chisel._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.util._

object CustomClientStates {
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
  def isValid(dummy: Int = 0): Bool = state < CustomClientStates.ISrr

  /** Determine whether this cmd misses, and the new state (on hit) or param to be sent (on miss) */
  private def growStarter(cmd: UInt): (Bool, UInt) = {
    import CustomMemoryOpCategories._
    import TLPermissions._
    import CustomClientStates._
    val c = categorize(cmd)
    val wasHit = Wire(Bool())
    wasHit := (c === rd && state < ISgd) || (c === wr && state < SMg)
    val nextState = Wire(UInt())
    when (c === wr) {
        when (state === E) {
            nextState := M
        } .elsewhen (state === Er) {
            nextState := Mr
        } .elsewhen (state === Err) {
            nextState := Mrr
        } .otherwise {
            nextState := state
        }
    } .otherwise {
        nextState := state
    }
    (wasHit, nextState.asUInt)
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

    val nextState = Wire(UInt())
    when (param === toT) {
        when (state === SMg || state === IMg) {
            nextState := M
        } .elsewhen (state === SMgr || state === IMgr) {
            nextState := Mr
        } .elsewhen (state === SMgrr || state === IMgrr) {
            nextState := Mrr
        } .otherwise {
            assert(true, "grant toT was unexpected")
        }
    } .elsewhen (param === toB) {
        when (state === ISgd) {
            nextState := S
        } .elsewhen (state === ISgdr) {
            nextState := Sr
        } .elsewhen (state === ISgdrr) {
            nextState := ISrr
        } .otherwise {
            assert(true, "grant toB was unexpected")
        }
    } .otherwise {
        assert(true, "grant was not toT or toB")
    }

    nextState.asUInt
  }

  /** Does this cache have permissions on this block sufficient to perform op,
    * and what to do next (Acquire message param or updated metadata). */
  def onAccess(cmd: UInt): (Bool, UInt, CustomClientMetadata) = {
    val r = growStarter(cmd)
    (r._1, r._2, CustomClientMetadata(r._2))
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

  /** Determine what state to go to based on Probe param */
  private def shrinkHelper(param: UInt): (Bool, UInt, UInt) = {
      //JAMESTODO: CURRENTLY IGNORES STALL CASES
    import CustomClientStates._
    import TLPermissions._
    val hasDirtyData = Wire(Bool())
    hasDirtyData := state < E
    val resp = Wire(UInt())
    val nextState = Wire(UInt())
    when (param === toB) {
        when (state === M || state === E) {
            nextState := S
            resp := TtoB
        } .elsewhen (state === Mr || state === Er) {
            nextState := Sr
            resp := TtoB
        } .elsewhen (state === SMg || state === SMgr || state === S || state === Sr) {
            resp := BtoB
            nextState := state
        } .otherwise {
            nextState := state
            resp := NtoN
        }
    } .elsewhen (param === toN) {
        when (state === M || state === E) {
            nextState := I
            resp := TtoN
        } .elsewhen(state === S) {
            nextState := I
            resp := BtoN
        } .elsewhen (state === Mr || state === Er) {
            nextState := Ir
            resp := TtoN
        } .elsewhen (state === Sr) {
            nextState := Ir
            resp := BtoN
        } .elsewhen (state === SMg) {
            nextState := IMg
            resp := BtoN
        } .elsewhen (state === SMgr) {
            nextState := IMgr
            resp := BtoN
        } .otherwise {
            nextState := state
            resp := NtoN
        }
    } .otherwise {
        assert (true, "toB or toN expected from probe")
    }
    (hasDirtyData, resp, nextState)
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
