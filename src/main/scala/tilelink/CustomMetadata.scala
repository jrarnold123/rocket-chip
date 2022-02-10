/*
  Copyright (c) 2021.  Nicolai Oswald
  Copyright (c) 2021.  University of Edinburgh
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met: redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer;
  redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution;
  neither the name of the copyright holders nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.tilelink

import Chisel._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.util._

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
object CustomClientStates {
  val width = 4
  def S_storePrefetch  = UInt(1, width)
  def S_store  = UInt(2, width)
  def S_evict_x_I  = UInt(3, width)
  def S_evict  = UInt(4, width)
  def S  = UInt(5, width)
  def M_evict_x_I  = UInt(6, width)
  def M_evict  = UInt(7, width)
  def M  = UInt(8, width)
  def I_storePrefetch  = UInt(9, width)
  def I_store  = UInt(10, width)
  def I_load  = UInt(11, width)
  def I  = UInt(0, width)
  def E_evict_x_I  = UInt(13, width)
  def E_evict  = UInt(14, width)
  def E  = UInt(15, width)

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

  /** Translate cache control cmds into Probe param */
  private def cmdToPermCap(cmd: UInt): UInt = {
    import CustomMemoryOpCategories._
    import TLPermissions._
    MuxLookup(cmd, toN, Seq(
      M_FLUSH   -> toN,
      M_PRODUCE -> toB,
      M_CLEAN   -> toT))
  }

  /** Is the block's data present in this cache */
  def isValid(dummy: Int = 0): Bool = state =/= CustomClientStates.I

  def isStable(): Bool = (state === CustomClientStates.E || state === CustomClientStates.I || state === CustomClientStates.M || state === CustomClientStates.S)
  
  def hasReadPermission(): (Bool, Bool) = {
    import CustomClientStates._
    MuxTLookup(state, (Bool(false),Bool(true)), Seq(
      S_storePrefetch    -> (Bool(false), Bool(true)),
      E    -> (Bool(true), Bool(false)),
      E_evict    -> (Bool(false), Bool(true)),
      M    -> (Bool(true), Bool(false)),
      E_evict_x_I    -> (Bool(false), Bool(true)),
      M_evict    -> (Bool(false), Bool(true)),
      I_load    -> (Bool(false), Bool(true)),
      S_evict_x_I    -> (Bool(false), Bool(true)),
      I_store    -> (Bool(false), Bool(true)),
      M_evict_x_I    -> (Bool(false), Bool(true)),
      I_storePrefetch    -> (Bool(false), Bool(true)),
      S_store    -> (Bool(false), Bool(true)),
      S_evict    -> (Bool(false), Bool(true)),
      I    -> (Bool(false), Bool(false)),
      S    -> (Bool(true), Bool(false))
    ))
  }
  
  def hasWritePermission(): (Bool, Bool) = {
    import CustomClientStates._
    MuxTLookup(state, (Bool(false), Bool(true)), Seq(
      S_storePrefetch    -> (Bool(false), Bool(true)),
      E    -> (Bool(true), Bool(false)),
      E_evict    -> (Bool(false), Bool(true)),
      M    -> (Bool(true), Bool(false)),
      E_evict_x_I    -> (Bool(false), Bool(true)),
      M_evict    -> (Bool(false), Bool(true)),
      I_load    -> (Bool(false), Bool(true)),
      S_evict_x_I    -> (Bool(false), Bool(true)),
      I_store    -> (Bool(false), Bool(true)),
      M_evict_x_I    -> (Bool(false), Bool(true)),
      I_storePrefetch    -> (Bool(false), Bool(true)),
      S_store    -> (Bool(false), Bool(true)),
      S_evict    -> (Bool(false), Bool(true)),
      I    -> (Bool(false), Bool(false)),
      S    -> (Bool(false), Bool(false))
    ))
  }
  /** Metadata change on a returned Grant */
  def onGrant(param: UInt): CustomClientMetadata = {
    import TLPermissions._
    import CustomClientStates._
    
    MuxTLookup(Cat(state, param), (Bool(false), UInt(0), CustomClientMetadata(I)), Seq(
      Cat(S_storePrefetch, toT)  ->  (Bool(false), UInt(0), CustomClientMetadata(E)),
      Cat(I_load, toB)  ->  (Bool(false), UInt(0), CustomClientMetadata(S)),
      Cat(I_store, toT)  ->  (Bool(false), UInt(0), CustomClientMetadata(M)),
      Cat(I_storePrefetch, toT)  ->  (Bool(false), UInt(0), CustomClientMetadata(E)),
      Cat(S_store, toT)  ->  (Bool(false), UInt(0), CustomClientMetadata(M))
    ))._3
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
  def onCacheControl(cmd: UInt): (Bool, UInt, CustomClientMetadata) = {
    import CustomClientStates._
    import TLPermissions._
    val param = cmdToPermCap(cmd)
    MuxTLookup(Cat(param, state), (Bool(false), UInt(0), CustomClientMetadata(I)), Seq(
      Cat(toN, E)    ->  (Bool(false), TtoN, CustomClientMetadata(E_evict)),
      Cat(toN, M)    ->  (Bool(true), TtoN, CustomClientMetadata(M_evict)),
      Cat(toN, S)    ->  (Bool(false), BtoN, CustomClientMetadata(S_evict))
    ))
  }
  def onProbe(param: UInt): (Bool, UInt, CustomClientMetadata) = {
    import TLPermissions._
    import CustomClientStates._
    MuxTLookup(Cat(param, state), (Bool(false), UInt(0), CustomClientMetadata(I)), Seq(
      Cat(toB, E)  ->  (Bool(false), TtoB, CustomClientMetadata(S)),
      Cat(toN, E)  ->  (Bool(false), TtoN, CustomClientMetadata(I)),
      Cat(toN, M)  ->  (Bool(true), TtoN, CustomClientMetadata(I)),
      Cat(toB, M)  ->  (Bool(true), TtoB, CustomClientMetadata(S)),
      Cat(toN, I)  ->  (Bool(false), NtoN, CustomClientMetadata(I)),
      Cat(toB, I)  ->  (Bool(false), NtoN, CustomClientMetadata(I)),
      Cat(toB, S)  ->  (Bool(false), BtoB, CustomClientMetadata(S)),
      Cat(toN, S)  ->  (Bool(false), BtoN, CustomClientMetadata(I))
    ))
  }
  def onMiss(cmd: UInt): (CustomClientMetadata, UInt) = {
    import TLPermissions._
    import CustomClientStates._
    import CustomMemoryOpCategories._
    
    val c = categorize(cmd)
    
    val out = MuxTLookup(Cat(c, state), (UInt(0), UInt(0)), Seq(
      Cat(wr, I)    ->  (I_store, NtoT),
      Cat(wi, I)    ->  (I_storePrefetch, NtoT),
      Cat(rd, I)    ->  (I_load, NtoB), 
      Cat(wr, S)    ->  (S_store, BtoT),
      Cat(wi, S)    ->  (S_storePrefetch, BtoT)
    ))
    (CustomClientMetadata(out._1), out._2)
  }
}












// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

/*package freechips.rocketchip.tilelink

import Chisel._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.util._

object CustomClientStates {
  val width = 4
  def I = UInt(0, width)
  def IS = UInt(1, width)
  def IE = UInt(2, width)
  def IM = UInt(3, width)
  def S = UInt(4, width)
  def SE = UInt(5, width)
  def SM = UInt(6, width)
  def E = UInt(7, width)
  def M = UInt(8, width)
  def MI = UInt(9, width) 
  def EI = UInt(10, width)
  def SI = UInt(11, width) 
  def II = UInt(12, width) //not implemented yet
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

*//** Stores the client-side coherence information,
  * such as permissions on the data and whether the data is dirty.
  * Its API can be used to make TileLink messages in response to
  * memory operations, cache control oeprations, or Probe messages.
  *//*
class CustomClientMetadata extends Bundle {
  // Actual state information stored in this bundle
  val state = UInt(width = CustomClientStates.width)

  // Metadata equality
  def ===(rhs: UInt): Bool = state === rhs
  def ===(rhs: CustomClientMetadata): Bool = state === rhs.state
  def =/=(rhs: CustomClientMetadata): Bool = !this.===(rhs)

  // Is the block's data present in this cache
  def isValid(dummy: Int = 0): Bool = state =/= CustomClientStates.I

  def isStable(): Bool = (state === CustomClientStates.M || state === CustomClientStates.E || state === CustomClientStates.S || state === CustomClientStates.I)

  def hasReadPermission(): (Bool, Bool) = {
    import CustomClientStates._
    MuxTLookup(state, (Bool(false),Bool(true)), Seq( //JamesToDo: consider changing default to false false?
              //canRead     //don't send request
      I     -> (Bool(false), Bool(false)),
      IS    -> (Bool(false), Bool(true)),
      IE    -> (Bool(false), Bool(true)),
      IM    -> (Bool(false), Bool(true)),
      S     -> (Bool(true), Bool(false)),
      SE    -> (Bool(true), Bool(true)),
      SM    -> (Bool(true), Bool(true)),
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
      IE    -> (Bool(false), Bool(true)),
      IM    -> (Bool(false), Bool(true)),
      S     -> (Bool(false), Bool(false)),
      SE    -> (Bool(false), Bool(true)),
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
      Cat(wi, I)    -> (IE, NtoT),
      Cat(rd, I)    -> (IS, NtoB),
      Cat(wr, S)    -> (SM, BtoT),
      Cat(wi, S)    -> (SE, BtoT)))
    
    assert(c === wr || c === wi || c=== rd)
    assert(state =/= M && state =/= E)
    assert(state =/= IM)
    assert(state =/= IE)
    assert(state =/= IS)
    assert(state =/= SM)
    assert(state =/= SE)
    assert(state === S || state === I)
    assert(c =/= rd || state === I)
    assert(out._1 =/= I)
    
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

  // Metadata change on a returned Grant
  def onGrant(param: UInt): CustomClientMetadata = {
    import TLPermissions._
    import CustomClientStates._

    assert(state =/= S)
    assert(state =/= I)
    assert(state =/= M)
    assert(state =/= E)
    assert(state =/= MI)
    assert(state =/= SI)
    

    MuxTLookup(Cat(state, param), (Bool(false), UInt(0), CustomClientMetadata(I)), Seq(
      //state, param ->      (dummy,       dummy,   nextState)
      Cat(IE, toT)     ->      (Bool(false), UInt(0), CustomClientMetadata(E)),
      Cat(SE, toT)     ->      (Bool(false), UInt(0), CustomClientMetadata(E)),
      Cat(IM, toT)     ->      (Bool(false), UInt(0), CustomClientMetadata(M)),
      Cat(SM, toT)     ->      (Bool(false), UInt(0), CustomClientMetadata(M)),
      Cat(IS, toB)     ->      (Bool(false), UInt(0), CustomClientMetadata(S))
    ))._3
    
    
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

  // Translate cache control cmds into Probe param
  private def cmdToPermCap(cmd: UInt): UInt = {
    import CustomMemoryOpCategories._
    import TLPermissions._
    MuxLookup(cmd, toN, Seq(
      M_FLUSH   -> toN,
      M_PRODUCE -> toB,
      M_CLEAN   -> toT))
  }

  def onCacheControl(cmd: UInt): (Bool, UInt, CustomClientMetadata) = {
    *//*val r = onProbe(cmdToPermCap(cmd))
    (r._1, r._2, r._3)*//*
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
  

// Factories for CustomClientMetadata, including on reset
object CustomClientMetadata {
  def apply(perm: UInt) = {
    val meta = Wire(new CustomClientMetadata)
    meta.state := perm
    meta
  }
  def onReset = CustomClientMetadata(CustomClientStates.I)
  def maximum = CustomClientMetadata(CustomClientStates.M)
}
*/