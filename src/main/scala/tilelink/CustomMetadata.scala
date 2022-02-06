// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.tilelink

import Chisel._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.util._

object CustomClientStates {
  val width = 4
  def I = UInt(0, width)
  def IS = UInt(1, width)
  def IM = UInt(2, width)
  def S = UInt(3, width)
  def SM = UInt(4, width)
  def E = UInt(5, width)
  def M = UInt(6, width)
  def MI = UInt(7, width) 
  def EI = UInt(8, width)
  def SI = UInt(9, width) 
  def II = UInt(10, width) //not implemented yet
}

object CustomMemoryOpCategories extends MemoryOpConstants {
  def wr = 1.U   // Op actually writes
  def wi = 2.U  // Future op will write
  def rd = 3.U // Op only reads

  def categorize(cmd: UInt): UInt = {
    //had to change this around... Operations that aren't wr or wi should NOT be classified rd
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
    //assert(out.isOneOf(wr,wi,rd), "Could not categorize command.")
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
  def isValid(dummy: Int = 0): Bool = state =/= CustomClientStates.I

  def isStable(): Bool = (state === CustomClientStates.M || state === CustomClientStates.E || state === CustomClientStates.S || state === CustomClientStates.I)

  def isDirty(): Bool = {
    (state === CustomClientStates.M || state === CustomClientStates.MI)
  }

  def hasReadPermission(): (Bool, Bool) = {
    import CustomClientStates._
    MuxTLookup(state, (Bool(false),Bool(true)), Seq( //JamesToDo: consider changing default to false false?
              //canRead     //don't send request
      I     -> (Bool(false), Bool(false)),
      IS    -> (Bool(false), Bool(true)),
      IM    -> (Bool(false), Bool(true)),
      S     -> (Bool(true), Bool(false)),
      SM    -> (Bool(true), Bool(true)), //jamesToDid: modify newFSM to allow the second one to be true, also don't let transients read
      M     -> (Bool(true), Bool(false)),
      E     -> (Bool(true), Bool(false)),
      MI    -> (Bool(false), Bool(true)),
      EI    -> (Bool(false), Bool(true)),
      SI    -> (Bool(false), Bool(true)),
      II    -> (Bool(false), Bool(true))
    ))
  }

  def hasWritePermission(): (Bool, Bool) = {
    import CustomClientStates._
    MuxTLookup(state, (Bool(false), Bool(true)), Seq(
              //canWrite    //mustStall
      I     -> (Bool(false), Bool(false)),
      IS    -> (Bool(false), Bool(true)),
      IM    -> (Bool(false), Bool(true)),
      S     -> (Bool(false), Bool(false)),
      SM    -> (Bool(false), Bool(true)),
      M     -> (Bool(true), Bool(false)),
      E     -> (Bool(true), Bool(false)),
      MI    -> (Bool(false), Bool(true)),
      EI    -> (Bool(false), Bool(true)),
      SI    -> (Bool(false), Bool(true)),
      II    -> (Bool(false), Bool(true))
    ))
  }

  def onMiss(cmd: UInt): (CustomClientMetadata, UInt) = {
    import CustomClientStates._
    import CustomMemoryOpCategories._
    import TLPermissions._

    val c = categorize(cmd)

    //NOTE: THE WRITES MUST BE CHECKED BEFORE READS
    //JamesTODO: check if the above is still true now that Categorize() has been fixed
    val out = MuxTLookup(Cat(c, state), (UInt(0), UInt(0)), Seq(
      Cat(wr, I)    -> (IM, NtoT),
      Cat(wi, I)    -> (IM, NtoT),
      Cat(rd, I)    -> (IS, NtoB),
      Cat(wr, S)    -> (SM, BtoT),
      Cat(wi, S)    -> (SM, BtoT)))
    
    (CustomClientMetadata(out._1), out._2)
  }

  //jamesTODO: change to MuxTLookup with dummy value as second return
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
    /*val r = onProbe(cmdToPermCap(cmd))
    (r._1, r._2, r._3)*/
    import CustomClientStates._
    import TLPermissions._
    val param = cmdToPermCap(cmd)
    assert(param === toN)
    MuxTLookup(Cat(param, state), (Bool(false), UInt(0), CustomClientMetadata(I)), Seq(
      //(param,state) ->      (dirty, param, next state)
      Cat(toN, M)     ->      (Bool(true),  TtoN, CustomClientMetadata(MI)),
      Cat(toN, E)     ->      (Bool(false), TtoN, CustomClientMetadata(EI)),
      Cat(toN, S)     ->      (Bool(false), BtoN, CustomClientMetadata(SI))))
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
