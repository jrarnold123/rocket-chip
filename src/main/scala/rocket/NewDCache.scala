/*package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.amba._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.diplomaticobjectmodel.model.OMSRAM
import freechips.rocketchip.tile.{CoreBundle, LookupByHartId}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import chisel3.{DontCare, WireInit, dontTouch, withClock}
import chisel3.experimental.{chiselName, NoChiselNamePrefix}
import chisel3.internal.sourceinfo.SourceInfo
import TLMessages._

// TODO: delete this trait once deduplication is smart enough to avoid globally inlining matching circuits
trait InlineInstance { self: chisel3.experimental.BaseModule =>
  chisel3.experimental.annotate(
    new chisel3.experimental.ChiselAnnotation {
      def toFirrtl: firrtl.annotations.Annotation = firrtl.passes.InlineAnnotation(self.toNamed) } )
}

class DCacheErrors(implicit p: Parameters) extends L1HellaCacheBundle()(p)
    with CanHaveErrors {
  val correctable = (cacheParams.tagCode.canCorrect || cacheParams.dataCode.canCorrect).option(Valid(UInt(width = paddrBits)))
  val uncorrectable = (cacheParams.tagCode.canDetect || cacheParams.dataCode.canDetect).option(Valid(UInt(width = paddrBits)))
  val bus = Valid(UInt(width = paddrBits))
}

class DCacheDataReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val addr = Bits(width = untagBits)
  val write = Bool()
  val wdata = UInt(width = encBits * rowBytes / eccBytes)
  val wordMask = UInt(width = rowBytes / subWordBytes)
  val eccMask = UInt(width = wordBytes / eccBytes)
  val way_en = Bits(width = nWays)
}

// ! Cache Data
class DCacheDataArray(implicit p: Parameters) extends L1HellaCacheModule()(p) {
  val io = new Bundle {
    val req = Valid(new DCacheDataReq).flip
    val resp = Vec(nWays, UInt(width = req.bits.wdata.getWidth)).asOutput
  }

  require(rowBits % subWordBits == 0, "rowBits must be a multiple of subWordBits")
  val eccMask = if (eccBits == subWordBits) Seq(true.B) else io.req.bits.eccMask.asBools
  val wMask = if (nWays == 1) eccMask else (0 until nWays).flatMap(i => eccMask.map(_ && io.req.bits.way_en(i)))
  val wWords = io.req.bits.wdata.grouped(encBits * (subWordBits / eccBits))
  val addr = io.req.bits.addr >> rowOffBits
  val data_arrays = Seq.tabulate(rowBits / subWordBits) {
    i =>
      DescribedSRAM(
        name = s"data_arrays_${i}",
        desc = "DCache Data Array",
        size = nSets * cacheBlockBytes / rowBytes,
        data = Vec(nWays * (subWordBits / eccBits), UInt(width = encBits))
      )
  }

  val rdata = for (((array, omSRAM), i) <- data_arrays zipWithIndex) yield {
    val valid = io.req.valid && (Bool(data_arrays.size == 1) || io.req.bits.wordMask(i))
    when (valid && io.req.bits.write) {
      val wMaskSlice = (0 until wMask.size).filter(j => i % (wordBits/subWordBits) == (j % (wordBytes/eccBytes)) / (subWordBytes/eccBytes)).map(wMask(_))
      val wData = wWords(i).grouped(encBits)
      array.write(addr, Vec((0 until nWays).flatMap(i => wData)), wMaskSlice)
    }
    val data = array.read(addr, valid && !io.req.bits.write)
    data.grouped(subWordBits / eccBits).map(_.asUInt).toSeq
  }
  (io.resp zip rdata.transpose).foreach { case (resp, data) => resp := data.asUInt }
}

// ! Cache Metadata (way, addr, etc)
class DCacheMetadataReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val write = Bool()
  val addr = UInt(width = vaddrBitsExtended)
  val idx = UInt(width = idxBits)
  val way_en = UInt(width = nWays)
  val data = UInt(width = cacheParams.tagCode.width(new L1Metadata().getWidth))
}

// ! Encompassing object
class DCache(staticIdForMetadataUseOnly: Int, val crossing: ClockCrossingType)(implicit p: Parameters) extends HellaCache(staticIdForMetadataUseOnly)(p) {
  override lazy val module = new DCacheModule(this)
  override def getOMSRAMs(): Seq[OMSRAM] = Seq(module.dcacheImpl.omSRAM) ++ module.dcacheImpl.data.data_arrays.map(_._2)
}

class DCacheTLBPort(implicit p: Parameters) extends CoreBundle()(p) {
  val req = Flipped(Decoupled(new TLBReq(coreDataBytes.log2)))
  val s1_resp = Output(new TLBResp)
  val s2_kill = Input(Bool())
}

// ! ACTUAL MODULE IS HERE
@chiselName
class DCacheModule(outer: DCache) extends HellaCacheModule(outer) {

  val tECC = cacheParams.tagCode
  val dECC = cacheParams.dataCode
  require(subWordBits % eccBits == 0, "subWordBits must be a multiple of eccBits")
  require(eccBytes == 1 || !dECC.isInstanceOf[IdentityCode])
  require(cacheParams.silentDrop || cacheParams.acquireBeforeRelease, "!silentDrop requires acquireBeforeRelease")
  val usingRMW = eccBytes > 1 || usingAtomicsInCache
  val mmioOffset = outer.firstMMIO
  edge.manager.requireFifo(TLFIFOFixer.allVolatile)  // TileLink pipelining MMIO requests

  val clock_en_reg = Reg(Bool())
  io.cpu.clock_enabled := clock_en_reg

  val gated_clock =
    if (!cacheParams.clockGate) clock
    else ClockGate(clock, clock_en_reg, "dcache_clock_gate")
  @chiselName class DCacheModuleImpl extends NoChiselNamePrefix { // entering gated-clock domain
    
    tl_out_d.ready := tl_out.e.ready
    tl_out.b.ready := !tl_out.d.fire()
    io.cpu.req.ready := !tl_out.d.fire() && !tl_out.b.fire()

    val tlb = Module(new TLB(false, log2Ceil(coreDataBytes), TLBConfig(nTLBSets, nTLBWays, cacheParams.nTLBBasePageSectors, cacheParams.nTLBSuperpages)))
    val pma_checker = Module(new TLB(false, log2Ceil(coreDataBytes), TLBConfig(nTLBSets, nTLBWays, cacheParams.nTLBBasePageSectors, cacheParams.nTLBSuperpages)) with InlineInstance)

    // tags
    val replacer = ReplacementPolicy.fromString(cacheParams.replacementPolicy, nWays)
    val addrToUse = Wire(Bits(width = vaddrBitsExtended))
    val metaInputReady = Wire(Bool())
    val metaInputValid = Wire(Bool())
    metaInputReady := clock_en_reg // possibly always true?


    val (tag_array, omSRAM) = DescribedSRAM(
      name = "tag_array",
      desc = "DCache Tag Array",
      size = nSets,
      data = Vec(nWays, metaInput.data)
    )

    // data
    val data = Module(new DCacheDataArray)
    val dataInput = Module(new DCacheDataReq)
    dataInputValid = Wire(Bool())
    data.io.req.ready := true
    data.io.req.valid := dataInputValid
    data.io.req.bits := dataInput

    //ADDRESSES
    val vaddr = Cat(addrToUse >> blockOffBits, io.cpu.req.bits.addr(blockOffBits-1, 0))
    val paddr = vaddr(paddrBits-1, 0)


    //SETTING UP A QUEUE ON A OUTPUTS (probably not needed for our implementation)
    val tl_out_a = Wire(tl_out.a)
    tl_out.a <> {
      val a_queue_depth = outer.crossing match {
        case RationalCrossing(_) => // TODO make this depend on the actual ratio?
          if (cacheParams.separateUncachedResp) (maxUncachedInFlight + 1) / 2
          else 2 min maxUncachedInFlight-1
        case SynchronousCrossing(BufferParams.none) => 1 // Need some buffering to guarantee livelock freedom
        case SynchronousCrossing(_)  => 0 // Adequate buffering within the crossing
        case _: AsynchronousCrossing => 0 // Adequate buffering within the crossing
        case _: CreditedCrossing     => 0 // Adequate buffering within the crossing
      }
      Queue(tl_out_a, a_queue_depth, flow = true)
    }

    //QUEUE NOT USED FOR C MESSAGES IN THIS IMPLEMENTATION
    val (tl_out_c, release_queue_empty) = (tl_out.c, true.B)

    //D MESSAGE SETUP
    val (d_first, d_last, d_done, d_address_inc) = edge.addr_inc(tl_out.d)

    val tl_out_e = tl_out.e

    //INTERFACING WITH METADATA CACHE
    val meta = tag_array.read(metaIdx, metaInputValid, && !metaInput.write)
    val meta_uncorrected = meta.map(tECC.decode(_).uncorrected.asTypeOf(new L1Metadata))
    val meta_tag = paddr >> tagLSB
    val meta_hit_way = meta_uncorrected.map(r => r.coh.isValid() && r.tag == meta_tag).asUInt
    val meta_hit_state = CustomClientMetadata.onReset.fromBits(meta_uncorrected.map(r => Mux(r.tag === meta_tag && !flush_valid, r.coh.asUInt, UInt(0))).reduce (_|_)) // best I can tell, this says "either you hit and got the state, or you missed and got N/I"


    //MESSAGE TYPES
    val grantAck = edge.GrantAck(tl_out.d.bits)

    def newFSM() = {
      //tl_out_a.valid := false
      //tl_out_c.valid := false
      tl_out_e.valid := false
      metaValid := false
      val message = Wire(UInt())
      val state = Wire(CustomClientMetadata(CustomClientStates.I))
      message := selectMessageToHandle()
      when (message === 4.U) { //grant
        /*state := readMeta(/**/)
        when (state === ...) {
          sendMessage
          metaWrite(...)
        }*/
        sendEMessage(grantAck, d_first)
        writeMetaGrant(meta_hit_state.onGrant(tl_out.d.bits.param))
      } .elsewhen (message === 5.U) { //grantData
        sendEMessage(grantAck, d_first)
        writeMetaGrant(meta_hit_state.onGrant(tl_out.d.bits.param))
        writeDataGrant()
      } .elsewhen (message === 6.U) { //releaseAck

      } .elsewhen (message === 14.U) { //probeBlock

      } .elsewhen (message === 16.U) { //read
        when (!s2_victim_dirty && s2_valid_cached_miss) {
          sendAMessage(acquire(paddr, meta_hit_state.onAccess(io.cpu.req.bits.cmd)._2), true)
        } .otherwise {
          tl_out_a.valid := false.B
        }
      } .elsewhen (message === 17.U) { //write
        when (!s2_victim_dirty && s2_valid_cached_miss) {
          sendAMessage(acquire(paddr, meta_hit_state.onAccess(io.cpu.req.bits.cmd)._2), true)
        } .otherwise {
          tl_out_a.valid := false.B
        }
      }
    }

    def selectMessageToHandle(): UInt = {
      val out = Wire(UInt())
      out := 0
      when (tl_out.d.valid) {
        metaIdx := 
        out := tl_out.d.bits.opcode
      } .elsewhen (tl_out.b.valid) {
        out := 8.U + tl_out.b.bits.opcode
      } .elsewhen (io.cpu.req.valid && !io.cpu.s2_kill) {
        when (isRead(io.cpu.req.bits.cmd)) {
          out := 16.U
        } .otherwise {
          out := 17.U
        }
      }
      out
    }

    def writeDataCPUWrite() = {
      val valid = should_pstore_drain(false)
      val write = pstore_drain
      val addr = Mux(pstore2_valid, pstore2_addr, pstore1_addr)
      val way = Mux(pstore2_valid, pstore2_way, pstore1_way)
      val data = encodeData(Fill(rowWords, Mux(pstore2_valid, pstore2_storegen_data, pstore1_data)), false.B)
      val eccMask = eccMaskFunc(Mux(pstore2_valid, pstore2_storegen_mask, pstore1_mask))
      val wordMask = {
        val eccMaskInner = eccMask.asBools.grouped(subWordBytes/eccBytes).map(_.orR).toSeq.asUInt
        val wordMask = UIntToOH(Mux(pstore2_valid, pstore2_addr, pstore1_addr).extract(rowOffBits-1, wordBytes.log2))
        FillInterleaved(wordBytes/subWordBytes, wordMask) & Fill(rowBytes/wordBytes, eccMaskInner)
      }
      writeDataHelper(addr, way, data, wordMask, eccMask, write, valid, 0.U)
    }
    def writeDataGrant() = {
      val valid = tl_out.d.valid && grantIsRefill && canAcceptCachedGrant
      val write = true
      val addr =  (vaddr >> idxLSB) << idxLSB | d_address_inc
      val way = refill_way
      val data = tl_d_data_encoded
      val wordMask = ~UInt(0, rowBytes / subWordBytes)
      val eccMask = ~UInt(0, wordBytes / eccBytes)
      writeDataHelper(addr, way, data, wordMask, eccMask, write, valid, 1.U)
    }
    def writeDataProbe() = {
      val valid = inWriteback && releaseDataBeat < refillCycles
      val data = dataArb.io.in(1).bits.wdata
      val write = false
      val addr = (probeIdx(probe_bits) << blockOffBits) | (releaseDataBeat(log2Up(refillCycles)-1,0) << rowOffBits)
      val wordMask = ~UInt(0, rowBytes / subWordBytes)
      val eccMask = ~UInt(0, wordBytes / eccBytes)
      val way = ~UInt(0, nWays)
      writeDataHelper(addr, way, data, wordMask, eccMask, write, valid, 2.U)
    }
    def writeDataCPURead() = {
      val valid = io.cpu.req.valid && likelyNeedsRead(io.cpu.req.bits)
      val data = dataArb.io.in(1).bits.wdata
      val write = false
      val addr = io.cpu.req.bits.addr
      val wordMask = {
        val mask = (subWordBytes.log2 until rowOffBits).foldLeft(1.U) { case (in, i) =>
          val upper_mask = Mux(i >= wordBytes.log2 || io.cpu.req.bits.size <= i.U, 0.U,
            ((BigInt(1) << (1 << (i - subWordBytes.log2)))-1).U)
          val upper = Mux(io.cpu.req.bits.addr(i), in, 0.U) | upper_mask
          val lower = Mux(io.cpu.req.bits.addr(i), 0.U, in)
          upper ## lower
        }
        Fill(subWordBytes / eccBytes, mask)
      }
      val eccMask = ~UInt(0, wordBytes / eccBytes)
      val way = ~UInt(0, nWays)
      writeDataHelper(addr, way, data, wordMask, eccMask, write, valid, 3.U)
    }    
    def writeDataHelper(addr: UInt, way: UInt, data: UInt, wordMask: UInt, eccMask: UInt, write: Bool, valid: Bool, priority: UInt) = {
      dataInputValid := valid
      dataInput.write := write
      dataInput.addr := addr
      dataInput.way_en := way
      dataInput.wdata := data
      dataInput.wordMask := wordMask
      dataInput.eccMask := eccMask
    }

    def writeMetaReset() = {
      val valid = resetting
      val write = true
      val index = flushCounter(idxBits-1, 0)
      val addr = Cat(io.cpu.req.bits.addr >> untagBits, flushCounter(idxBits-1, 0) << blockOffBits)
      val way = ~UInt(0, nWays)
      val data = tECC.encode(CustomL1Metadata(0.U, CustomClientMetadata.onReset).asUInt)
      val wmask = if (nWays == 1) Seq(true.B) else way.asBools
      when (valid) {
        addrToUse := addr
        tag_array.write(index, Vec.fill(nWays)(data), wmask)
      }
    }
    
    def writeMetaHit() = {
      val valid = s2_valid_hit_pre_data_ecc_and_waw && s2_update_meta
      val write = true
      val way = Wire(Bits(width = nWays))
      way := s2_victim_or_hit_way
      val index = vaddr(idxMSB, idxLSB)
      val addr = Cat(io.cpu.req.bits.addr >> untagBits, vaddr(idxMSB, 0))
      //s0_req.addr := Cat(addr >> blockOffBits, io.cpu.req.bits.addr(blockOffBits-1,0))
      val data = tECC.encode(CustomL1Metadata(s2_req.addr >> tagLSB, s2_new_hit_state).asUInt)
      val wmask = if (nWays == 1) Seq(true.B) else way.asBools
      when (valid && write) {
        addrToUse := addr
        tag_array.write(index, Vec.fill(nWays)(data), wmask)
      } .elsewhen (valid) {
        addrToUse := addr
        meta := tag_array.read(index, true)
      }
    }
    def writeMetaGrant(newState: CustomClientMetadata) = {
      val valid = true
      val write = true
      val way = Wire(Bits(width = nWays))
      way := refill_way
      val index = tl_out.d.bits.address(idxMSB, idxLSB)
      val addr = tl_out.d.bits.address
      val data = tECC.encode(CustomL1Metadata(s2_req.addr >> tagLSB, newState).asUInt)//s2_hit_state.onGrant(s2_req.cmd, tl_out.d.bits.param)).asUInt)
      val wmask = if (nWays == 1) Seq(true.B) else way.asBools
      when (valid) {
        addrToUse := addr
        tag_array.write(index, Vec.fill(nWays)(data), wmask)
      }
    }
    def writeMetaRelease() = {
      val valid = release_state.isOneOf(s_voluntary_write_meta, s_probe_write_meta)
      val write = true
      probeWay := releaseWay
      val index = probeIdx(probe_bits)
      val addr = Cat(io.cpu.req.bits.addr >> untagBits, probe_bits.address(idxMSB, 0))
      probeData := tECC.encode(CustomL1Metadata(tl_out_c.bits.address >> tagLSB, newCoh).asUInt)
      val wmask = if (nWays ==1) Seq(true.B) else probeWay.asBools
      when (valid) {
        addrToUse := addr
        tag_array.write(index, Vec.fill(nWays)(probeData), wmask)
      }
    }
    def writeMetaProbe() = {
      val valid = release_state === s_probe_retry || (tl_out.b.valid && (!block_probe_for_core_progress || lrscBackingOff))
      val write = false
      val index = Mux(release_state === s_probe_retry, probeIdx(tl_out.b.bits), probeIdx(tl_out.b.bits))
      val addr = Cat(io.cpu.req.bits.addr >> paddrBits, Mux(release_state === s_probe_retry, probe_bits.address, tl_out.b.bits.address))
      val way = probeWay
      val data = probeData
      when (valid) {
        addrToUse := addr
        meta := tag_array.read(index, true)
      }
    }
    def writeMetaCPU() = {
      val valid = io.cpu.req.valid
      val write = false
      val index = io.cpu.req.bits.addr(idxMSB, idxLSB)
      val addr = io.cpu.req.bits.addr
      val way = probeWay
      val data = probeData
      when (valid) {
        addrToUse := addr
        meta := tag_array.read(index, true)
      }
    }

    def readMeta(idx: UInt) = {
      tag_array.read(idx, true)
    }



    def acquire(paddr: UInt, param: UInt): TLBundleA = {
    // ? Seems to check if AcquireB is supported by any slaves, then adjusts the message accordingly?
    if (!edge.manager.anySupportAcquireB) Wire(new TLBundleA(edge.bundle))
    else edge.AcquireBlock(UInt(0), paddr >> lgCacheBlockBytes << lgCacheBlockBytes, lgCacheBlockBytes, param)._2
  }
  }
}*/