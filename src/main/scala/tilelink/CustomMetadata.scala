// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.tilelink

import Chisel._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.util._

object CustomClientStates {
  /*val width = 2
  def I = UInt(0, width)
  def S  = UInt(1, width)
  def E   = UInt(2, width)
  def M   = UInt(3, width)
  */
  val width = 4
  def I = UInt(0, width)
  def ISd = UInt(1, width)
  def IMad = UInt(2, width)
  def IMa = UInt(3, width)
  def S = UInt(4, width)
  def SMad = UInt(5, width)
  def SMa = UInt(6, width)
  def E = UInt(7, width)
  def M = UInt(8, width)
  def MIa = UInt(9, width)
  def EIa = UInt(10, width)
  def SIa = UInt(11, width)
  def IIa = UInt(12, width)
}

object CustomMemoryOpCategories extends MemoryOpConstants {
  def wr = 1.U   // Op actually writes
  def wi = 2.U  // Future op will write
  def rd = 3.U // Op only reads

  def categorize(cmd: UInt): UInt = {
    val out = Wire(UInt())
    when (isWrite(cmd)) {
      out := wr
    } .elsewhen (isWriteIntent(cmd)) {
      out := wi
    } .elsewhen (isRead(cmd)) {
      out := rd
    } .otherwise {
      out := 0.U
    }
    assert(out.isOneOf(wr,wi,rd), "Could not categorize command.")
    out
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
  def isValid(dummy: Int = 0): Bool = state > CustomClientStates.I && state <= CustomClientStates.IIa

  def isStable(): Bool = (state === CustomClientStates.M || state === CustomClientStates.E || state === CustomClientStates.S || state === CustomClientStates.I)

  def isDirty(): Bool = {
    (state === CustomClientStates.M || state === CustomClientStates.MIa)
  }

  def hasReadPermission(): (Bool, Bool) = {
    import CustomClientStates._
    MuxTLookup(state, (Bool(false),Bool(true)), Seq( //JamesToDo: consider changing default to false false?
    //jamesToDo: currently the stall is unused
                                  //canRead     //mustStall
      I     -> (Bool(false), Bool(false)),
      ISd   -> (Bool(false), Bool(true)),
      IMad  -> (Bool(false), Bool(true)),
      //IMa   -> (Bool(false), Bool(true)),
      S     -> (Bool(true), Bool(false)),
      SMad  -> (Bool(true), Bool(false)),
      SMa   -> (Bool(true), Bool(false)),
      M     -> (Bool(true), Bool(false)),
      E     -> (Bool(true), Bool(false)),
      MIa   -> (Bool(false), Bool(true)),
      EIa   -> (Bool(false), Bool(true)),
      SIa   -> (Bool(false), Bool(true)),
      IIa   -> (Bool(false), Bool(true))
    ))
  }

  def hasWritePermission(): (Bool, Bool) = {
    import CustomClientStates._
    MuxTLookup(state, (Bool(false), Bool(true)), Seq(
                                  //canWrite    //mustStall
      I     -> (Bool(false), Bool(false)),
      ISd   -> (Bool(false), Bool(true)),
      IMad  -> (Bool(false), Bool(true)),
      //IMa   -> (Bool(false), Bool(true)),
      S     -> (Bool(false), Bool(false)),
      SMad  -> (Bool(false), Bool(true)),
      SMa   -> (Bool(false), Bool(true)),
      M     -> (Bool(true), Bool(false)),
      E     -> (Bool(true), Bool(false)),
      MIa   -> (Bool(false), Bool(true)),
      EIa   -> (Bool(false), Bool(true)),
      SIa   -> (Bool(false), Bool(true)),
      IIa   -> (Bool(false), Bool(true))
    ))
  }

  def onMiss(cmd: UInt): CustomClientMetadata = {
    import CustomClientStates._
    import CustomMemoryOpCategories._
    val c = categorize(cmd)

    val out = Wire(UInt())

    when (state === I) {
      when(c === rd) {
        out := ISd
        assert(c =/= wi && c =/= wr)
      } .elsewhen (c === wi || c === wr) {
        out := IMad
      } . otherwise {
        out := state
      }
    } .elsewhen (state === S) {
      when (c === wi || c === wr) {
        out := SMa
      } .otherwise {
        out := state
      }
    }

    /*val out = MuxLookup(Cat(c, state), UInt(0), Seq(
      Cat(rd, I)    -> ISd,
      Cat(wi, I)    -> IMad,
      Cat(wr, I)    -> IMad,
      Cat(wi, S)    -> SMa,
      Cat(wr, S)    -> SMa
    ))*/
    
    CustomClientMetadata(out)
  }

  def onHit(cmd: UInt): CustomClientMetadata = {
    import CustomClientStates._
    import CustomMemoryOpCategories._
    val c = categorize(cmd)

    CustomClientMetadata(MuxLookup(Cat(c, state), UInt(0), Seq(
      Cat(wi, E)    -> E,
      Cat(wr, E)    -> M
    )))
  }

  def onWrite(): CustomClientMetadata = {
    val temp = CustomClientMetadata(CustomClientStates.I)
    when (state === CustomClientStates.E) {
      temp := CustomClientMetadata(CustomClientStates.M)
    } .otherwise {
      temp := CustomClientMetadata(state)
    }
    temp
  }

  /** Metadata change on a returned Grant */
  def onGrant(param: UInt): CustomClientMetadata = {
    import TLPermissions._
    import CustomClientStates._
    val nextState = Wire(UInt())
    
    when (param === toB) {
      nextState := S
    } .otherwise {
      assert(!isCap(param) || param === toT)
      nextState := E
    }
    CustomClientMetadata(nextState)
  }

  def onProbe(param: UInt): (Bool, UInt, CustomClientMetadata) = {
    import CustomClientStates._
    import TLPermissions._
    MuxTLookup(Cat(param, state), (Bool(false), UInt(0), CustomClientMetadata(I)), Seq(
      //(param,state) ->      (dirty, param, next state)
      Cat(toT, M)     ->      (Bool(true),  TtoT, CustomClientMetadata(E)),
      Cat(toT, E)     ->      (Bool(false), TtoT, CustomClientMetadata(E)),
      Cat(toT, S)     ->      (Bool(false), BtoB, CustomClientMetadata(S)),
      Cat(toT, I)     ->      (Bool(false), NtoN, CustomClientMetadata(I)),
      Cat(toB, M)     ->      (Bool(true),  TtoB, CustomClientMetadata(S)),
      Cat(toB, E)     ->      (Bool(false), TtoB, CustomClientMetadata(S)),
      Cat(toB, S)     ->      (Bool(false), BtoB, CustomClientMetadata(S)),
      Cat(toB, I)     ->      (Bool(false), NtoN, CustomClientMetadata(I)),
      Cat(toN, M)     ->      (Bool(true),  TtoN, CustomClientMetadata(I)),
      Cat(toN, E)     ->      (Bool(false), TtoN, CustomClientMetadata(I)),
      Cat(toN, S)     ->      (Bool(false), BtoN, CustomClientMetadata(I)),
      Cat(toN, I)     ->      (Bool(false), NtoN, CustomClientMetadata(I))))
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
    val r = onProbe(cmdToPermCap(cmd))
    (r._1, r._2, r._3)
  }

}
  

/** Factories for CustomClientMetadata, including on reset */
object CustomClientMetadata {
  def apply(perm: UInt) = {
    val meta = Wire(new CustomClientMetadata)
    meta.state := perm
    meta
  }
  def onReset = CustomClientMetadata(CustomClientStates.I)
  def maximum = CustomClientMetadata(CustomClientStates.M)
}
