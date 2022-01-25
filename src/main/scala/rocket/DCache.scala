// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

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
  val data = UInt(width = cacheParams.tagCode.width(new CustomL1Metadata().getWidth))
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
  val tlb_port = IO(new DCacheTLBPort)

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

    //tl_out.a.valid := false

    val tlb = Module(new TLB(false, log2Ceil(coreDataBytes), TLBConfig(nTLBSets, nTLBWays, cacheParams.nTLBBasePageSectors, cacheParams.nTLBSuperpages)))
    val pma_checker = Module(new TLB(false, log2Ceil(coreDataBytes), TLBConfig(nTLBSets, nTLBWays, cacheParams.nTLBBasePageSectors, cacheParams.nTLBSuperpages)) with InlineInstance)

    // tags
    val replacer = ReplacementPolicy.fromString(cacheParams.replacementPolicy, nWays)
    val metaArb = Module(new Arbiter(new DCacheMetadataReq, 8) with InlineInstance)

    val (tag_array, omSRAM) = DescribedSRAM(
      name = "tag_array",
      desc = "DCache Tag Array",
      size = nSets,
      data = Vec(nWays, metaArb.io.out.bits.data)
    )

    // data
    val data = Module(new DCacheDataArray)
    val dataArb = Module(new Arbiter(new DCacheDataReq, 4) with InlineInstance)
    dataArb.io.in.tail.foreach(_.bits.wdata := dataArb.io.in.head.bits.wdata) // tie off write ports by default
    data.io.req <> dataArb.io.out
    dataArb.io.out.ready := true
    metaArb.io.out.ready := clock_en_reg

    // ! tl_out_a is just an alias of tl_out.a.   Not sure why it was done, but ok.
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

    val (tl_out_c, release_queue_empty) = // ! (tl_out.c, true.B)
      if (cacheParams.acquireBeforeRelease) { // ! false
        val q = Module(new Queue(tl_out.c.bits.cloneType, cacheDataBeats, flow = true))
        tl_out.c <> q.io.deq
        (q.io.enq, q.io.count === 0)
      } else {
        (tl_out.c, true.B)
      }

    val tl_out_e = tl_out.e



    // ! fire is "ready and valid" boolean, basically checking if there's a message waiting there
    val s1_valid = Reg(next=io.cpu.req.fire(), init=Bool(false))
    val probe_bits = RegEnable(tl_out.b.bits, tl_out.b.fire()) // TODO has data now :(
    assert(!tl_out.b.ready|| !tl_out_c.valid)
    val s1_nack = Wire(init=Bool(false))
    val s1_valid_masked = s1_valid && !io.cpu.s1_kill
    val s1_valid_not_nacked = s1_valid && !s1_nack
    val s1_tlb_req_valid = RegNext(tlb_port.req.fire(), false.B)
    val s2_tlb_req_valid = RegNext(s1_tlb_req_valid, false.B)
    val s0_clk_en = metaArb.io.out.valid && !metaArb.io.out.bits.write

    val s0_req = WireInit(io.cpu.req.bits) // ! incoming request for stage 0
    s0_req.addr := Cat(metaArb.io.out.bits.addr >> blockOffBits, io.cpu.req.bits.addr(blockOffBits-1,0)) // ? extend address with virtual bits
    s0_req.idx.foreach(_ := Cat(metaArb.io.out.bits.idx, s0_req.addr(blockOffBits-1, 0))) // ? update the s0_req index
    when (!metaArb.io.in(7).ready) { s0_req.phys := true } // ? don't accept CPU access if it can't be processed quickly
    val s1_req = RegEnable(s0_req, s0_clk_en) // ! port s0 to s1 (stage latch)
    val s1_req_valid = RegEnable(io.cpu.req.valid, s0_clk_en)
    val s1_vaddr = s1_req.addr

    val s0_tlb_req = WireInit(tlb_port.req.bits)
    when (!tlb_port.req.fire()) {
      // ! if the tlb port hasn't fired yet then we update the info in s0_tlb_req
      s0_tlb_req.passthrough := s0_req.phys
      s0_tlb_req.vaddr := s0_req.addr
      s0_tlb_req.size := s0_req.size
      s0_tlb_req.cmd := s0_req.cmd
    }
    // ! move to next stage at the next clock cycle or once tlb verifies data is sent
    val s1_tlb_req = RegEnable(s0_tlb_req, s0_clk_en || tlb_port.req.valid)

    val s1_read = isRead(s1_req.cmd)
    val s1_write = isWrite(s1_req.cmd)
    val s1_readwrite = s1_read || s1_write
    val s1_sfence = s1_req.cmd === M_SFENCE
    val s1_flush_line = s1_req.cmd === M_FLUSH_ALL && s1_req.size(0)
    val s1_flush_valid = Reg(Bool())
    val s1_waw_hazard = Wire(Bool())

    val s_ready :: s_voluntary_writeback :: s_probe_rep_dirty :: s_probe_rep_clean :: s_probe_retry :: s_probe_rep_miss :: s_voluntary_write_meta :: s_probe_write_meta :: s_dummy :: s_voluntary_release :: s_voluntary_ack_write_meta :: Nil = Enum(UInt(), 11)
    val supports_flush = outer.flushOnFenceI || coreParams.haveCFlush
    val flushed = Reg(init=Bool(true))
    val flushing = Reg(init=Bool(false))
    val flushing_req = Reg(s1_req)
    val cached_grant_wait = Reg(init=Bool(false))
    val resetting = RegInit(false.B)
    val flushCounter = Reg(init=UInt(nSets * (nWays-1), log2Ceil(nSets * nWays)))
    val release_ack_wait = Reg(init=Bool(false))
    val release_ack_addr = Reg(UInt(paddrBits.W))
    val release_state = Reg(init=s_ready)
    val refill_way = Reg(UInt())
    val any_pstore_valid = Wire(Bool())
    val inWriteback = release_state.isOneOf(s_voluntary_writeback, s_probe_rep_dirty)
    val releaseWay = Wire(UInt())
    io.cpu.req.ready := (release_state === s_ready) && !cached_grant_wait && !s1_nack

    // I/O MSHRs
    val uncachedInFlight = RegInit(Vec.fill(maxUncachedInFlight)(false.B))
    val uncachedReqs = Reg(Vec(maxUncachedInFlight, new HellaCacheReq))
    val uncachedResp = WireInit(new HellaCacheReq, DontCare)

    // hit initiation path
    val s0_read = isRead(io.cpu.req.bits.cmd)
    readDataCPU()
 
    when (!dataArb.io.in(3).ready && s0_read) { io.cpu.req.ready := false }
    val s1_did_read = RegEnable(dataArb.io.in(3).ready && (io.cpu.req.valid && needsRead(io.cpu.req.bits)), s0_clk_en)
    val s1_read_mask = RegEnable(dataArb.io.in(3).bits.wordMask, s0_clk_en)
    val probeWay = Wire(Bits(width = nWays))
    val probeData = Wire(UInt())
    readMetaCPU()

    when (!metaArb.io.in(7).ready) { io.cpu.req.ready := false }

    // address translation
    // ! using TLB
    val s1_cmd_uses_tlb = s1_readwrite || s1_flush_line || s1_req.cmd === M_WOK
    io.ptw <> tlb.io.ptw
    tlb.io.kill := io.cpu.s2_kill || s2_tlb_req_valid && tlb_port.s2_kill
    tlb.io.req.valid := s1_tlb_req_valid || s1_valid && !io.cpu.s1_kill && s1_cmd_uses_tlb
    tlb.io.req.bits := s1_tlb_req
    when (!tlb.io.req.ready && !tlb.io.ptw.resp.valid && !io.cpu.req.bits.phys) { io.cpu.req.ready := false }
    when (!s1_tlb_req_valid && s1_valid && s1_cmd_uses_tlb && tlb.io.resp.miss) { s1_nack := true }

    tlb.io.sfence.valid := s1_valid && !io.cpu.s1_kill && s1_sfence
    tlb.io.sfence.bits.rs1 := s1_req.size(0)
    tlb.io.sfence.bits.rs2 := s1_req.size(1)
    tlb.io.sfence.bits.asid := io.cpu.s1_data.data
    tlb.io.sfence.bits.addr := s1_req.addr

    tlb_port.req.ready := clock_en_reg
    tlb_port.s1_resp := tlb.io.resp
    when (s1_tlb_req_valid && s1_valid && !(s1_req.phys && s1_req.no_xcpt)) { s1_nack := true }

    pma_checker.io.req.bits.passthrough := true
    pma_checker.io.req.bits := s1_req


    // ! UPDATE METADATA
    val s1_paddr = Cat(Mux(s1_tlb_req_valid, s1_req.addr(paddrBits-1, pgIdxBits), tlb.io.resp.paddr >> pgIdxBits), s1_req.addr(pgIdxBits-1, 0))
    val s1_victim_way = Wire(UInt())    
    val (s1_hit_way, s1_hit_state, s1_meta) =
      if (usingDataScratchpad) { // ! IGNORE
        val baseAddr = p(LookupByHartId)(_.dcache.flatMap(_.scratch.map(_.U)), io_hartid.get) | io_mmio_address_prefix.get
        val inScratchpad = s1_paddr >= baseAddr && s1_paddr < baseAddr + nSets * cacheBlockBytes
        val hitState = Mux(inScratchpad, CustomClientMetadata.maximum, CustomClientMetadata.onReset)
        val dummyMeta = CustomL1Metadata(UInt(0), CustomClientMetadata.onReset)
        (inScratchpad, hitState, Seq(tECC.encode(dummyMeta.asUInt)))
      } else { // ! ONLY USE THIS CASE
        val metaReq = metaArb.io.out
        val metaIdx = metaReq.bits.idx
        when (metaReq.valid && metaReq.bits.write) {
          val wmask = if (nWays == 1) Seq(true.B) else metaReq.bits.way_en.asBools
          tag_array.write(metaIdx, Vec.fill(nWays)(metaReq.bits.data), wmask)
        }
        val s1_meta = tag_array.read(metaIdx, metaReq.valid && !metaReq.bits.write)
        val s1_meta_uncorrected = s1_meta.map(tECC.decode(_).uncorrected.asTypeOf(new CustomL1Metadata))
        val s1_tag = s1_paddr >> tagLSB
        val s1_meta_hit_way = s1_meta_uncorrected.map(r => r.coh.isValid() && r.tag === s1_tag).asUInt
        val s1_meta_hit_state = CustomClientMetadata.onReset.fromBits(
          s1_meta_uncorrected.map(r => Mux(r.tag === s1_tag, r.coh.asUInt, UInt(0)))
          .reduce (_|_))
        val s1_meta_decoded_inner = s1_meta.map(tECC.decode(_))
        (s1_meta_hit_way, s1_meta_hit_state, s1_meta)
      }
    val s2_probe_stall = Wire(Bool()) //jamesToDo
    val s2_probe_stall_latch = Reg(next=s2_probe_stall,init=Bool(false))
    val s1_probe = RegEnable(next=tl_out.b.fire(), init=Bool(false), !s2_probe_stall && !s2_probe_stall_latch)
    val s2_probe_way = RegEnable(s1_hit_way, s1_probe && !s2_probe_stall_latch)
    val s2_probe_state = RegEnable(s1_hit_state, init=CustomClientMetadata(CustomClientStates.I), s1_probe && !s2_probe_stall_latch) //jamesToDid: adding an initial state
    assert(!s1_probe || s1_hit_state =/= CustomClientMetadata(CustomClientStates.SM))
    when (s1_probe && !s2_probe_stall_latch) {
      s2_probe_stall := !s1_hit_state.isStable()
    } .elsewhen (s2_probe_stall_latch) {
      assert(Bool(false)) //please don't let this work
      assert(s2_probe_state === CustomClientMetadata(CustomClientStates.SM))
      s2_probe_stall := !s2_probe_state.isStable()
    } .otherwise {
      s2_probe_stall := Bool(false)
    }
    assert(!s2_probe_stall)
    assert(!s2_probe_stall_latch)
    assert(!s1_probe || s1_hit_state =/= CustomClientMetadata(CustomClientStates.SM))


    val s1_data_way = Wire(init = if (nWays == 1) 1.U else Mux(inWriteback, releaseWay, s1_hit_way))
    val tl_d_data_encoded = Wire(encodeData(tl_out.d.bits.data, false.B).cloneType)
    val s1_all_data_ways = Vec(data.io.resp ++ (!cacheParams.separateUncachedResp).option(tl_d_data_encoded))
    val s1_mask_xwr = new StoreGen(s1_req.size, s1_req.addr, UInt(0), wordBytes).mask
    val s1_mask = Mux(s1_req.cmd === M_PWR, io.cpu.s1_data.mask, s1_mask_xwr)
    // for partial writes, s1_data.mask must be a subset of s1_mask_xwr
    assert(!(s1_valid_masked && s1_req.cmd === M_PWR) || (s1_mask_xwr | ~io.cpu.s1_data.mask).andR)

    val s2_valid = Reg(next=s1_valid_masked && !s1_sfence, init=Bool(false))
    val s2_valid_no_xcpt = s2_valid && !io.cpu.s2_xcpt.asUInt.orR
    val s2_probe = RegEnable(s1_probe, init=Bool(false), !s2_probe_stall)

    val releaseInFlight = s1_probe || s2_probe || release_state =/= s_ready
    val s2_not_nacked_in_s1 = RegNext(!s1_nack)
    val s2_valid_not_nacked_in_s1 = s2_valid && s2_not_nacked_in_s1
    val s2_valid_masked = s2_valid_no_xcpt && s2_not_nacked_in_s1
    val s2_valid_not_killed = s2_valid_masked && !io.cpu.s2_kill
    val s2_req = Reg(io.cpu.req.bits)
    val s2_req_valid = Reg(io.cpu.req.valid)
    val s2_cmd_flush_all = s2_req.cmd === M_FLUSH_ALL && !s2_req.size(0)
    val s2_cmd_flush_line = s2_req.cmd === M_FLUSH_ALL && s2_req.size(0)
    val s2_tlb_xcpt = Reg(tlb.io.resp.cloneType)
    val s2_pma = Reg(tlb.io.resp.cloneType)
    val s2_uncached_resp_addr = Reg(s2_req.addr.cloneType) // should be DCE'd in synthesis
    when (s1_valid_not_nacked) { // ? Keep the pipeline moving (unless s1 was nacked)
      s2_req := s1_req
      s2_req_valid := s1_req_valid
      s2_req.addr := s1_paddr
      s2_tlb_xcpt := tlb.io.resp
      s2_pma := Mux(s1_tlb_req_valid, pma_checker.io.resp, tlb.io.resp)
    } .otherwise {
      s2_req_valid := Bool(false) //apparently the initilization doesn't work
    }
    val s2_vaddr = Cat(RegEnable(s1_vaddr, s1_valid_not_nacked) >> tagLSB, s2_req.addr(tagLSB-1, 0))
    val s2_read = isRead(s2_req.cmd)
    val s2_write = isWrite(s2_req.cmd)
    val s2_readwrite = s2_read || s2_write
    val s1_meta_decoded = s1_meta.map(tECC.decode(_))
    val s1_meta_clk_en = s1_valid_not_nacked || s1_probe
    val s2_meta_correctable_errors = s1_meta_decoded.map(m => RegEnable(m.correctable, s1_meta_clk_en)).asUInt
    val s2_meta_uncorrectable_errors = s1_meta_decoded.map(m => RegEnable(m.uncorrectable, s1_meta_clk_en)).asUInt
    val s2_meta_error_uncorrectable = s2_meta_uncorrectable_errors.orR
    val s2_meta_corrected = s1_meta_decoded.map(m => RegEnable(m.corrected, s1_meta_clk_en).asTypeOf(new CustomL1Metadata))
    val s2_meta_error = (s2_meta_uncorrectable_errors | s2_meta_correctable_errors).orR
    val s2_data = {
      val wordsPerRow = rowBits / subWordBits
      val en = s1_valid || inWriteback || io.cpu.replay_next
      val word_en = Mux(inWriteback, Fill(wordsPerRow, 1.U), Mux(s1_did_read, s1_read_mask, 0.U))
      val s1_way_words = s1_all_data_ways.map(_.grouped(dECC.width(eccBits) * (subWordBits / eccBits)))
      if (cacheParams.pipelineWayMux) {
        val s1_word_en = Mux(io.cpu.replay_next, 0.U, word_en)
        (for (i <- 0 until wordsPerRow) yield {
          val s2_way_en = RegEnable(Mux(s1_word_en(i), s1_data_way, 0.U), en)
          val s2_way_words = (0 until nWays).map(j => RegEnable(s1_way_words(j)(i), en && word_en(i)))
          (0 until nWays).map(j => Mux(s2_way_en(j), s2_way_words(j), 0.U)).reduce(_|_)
        }).asUInt
      } else {
        val s1_word_en = Mux(!io.cpu.replay_next, word_en, UIntToOH(uncachedResp.addr.extract(log2Up(rowBits/8)-1, log2Up(wordBytes)), wordsPerRow))
        (for (i <- 0 until wordsPerRow) yield {
          RegEnable(Mux1H(Mux(s1_word_en(i), s1_data_way, 0.U), s1_way_words.map(_(i))), en)
        }).asUInt
      }
    }
    val s2_hit_way = RegEnable(s1_hit_way, s1_valid_not_nacked)
    val s2_hit_state = RegEnable(s1_hit_state, s1_hit_state, s1_valid_not_nacked)
    val s2_waw_hazard = RegEnable(s1_waw_hazard, s1_valid_not_nacked)
    val s2_store_merge = Wire(Bool())
    val s2_hit_valid = s2_hit_state.isValid()
    val s2_hit = Wire(Bool())
    val s2_new_hit_state = Wire(CustomClientMetadata(CustomClientStates.I))
    val s2_data_decoded = decodeData(s2_data)
    val s2_word_idx = s2_req.addr.extract(log2Up(rowBits/8)-1, log2Up(wordBytes))
    val s2_data_error = s2_data_decoded.map(_.error).orR
    val s2_data_error_uncorrectable = s2_data_decoded.map(_.uncorrectable).orR
    val s2_data_corrected = (s2_data_decoded.map(_.corrected): Seq[UInt]).asUInt
    val s2_data_uncorrected = (s2_data_decoded.map(_.uncorrected): Seq[UInt]).asUInt
    val s2_valid_hit_maybe_flush_pre_data_ecc_and_waw = s2_valid_masked && !s2_meta_error && s2_hit
    val s2_no_alloc_hazard = if (!usingVM || pgIdxBits >= untagBits) false.B else {
      // make sure that any in-flight non-allocating accesses are ordered before
      // any allocating accesses.  this can only happen if aliasing is possible.
      val any_no_alloc_in_flight = Reg(Bool())
      when (!uncachedInFlight.asUInt.orR) { any_no_alloc_in_flight := false }
      when (s2_valid && s2_req.no_alloc) { any_no_alloc_in_flight := true }
      val s1_need_check = any_no_alloc_in_flight || s2_valid && s2_req.no_alloc

      val concerns = (uncachedInFlight zip uncachedReqs) :+ (s2_valid && s2_req.no_alloc, s2_req)
      val s1_uncached_hits = concerns.map { c =>
        val concern_wmask = new StoreGen(c._2.size, c._2.addr, UInt(0), wordBytes).mask
        val addr_match = (c._2.addr ^ s1_paddr)(pgIdxBits+pgLevelBits-1, wordBytes.log2) === 0
        val mask_match = (concern_wmask & s1_mask_xwr).orR || c._2.cmd === M_PWR || s1_req.cmd === M_PWR
        val cmd_match = isWrite(c._2.cmd) || isWrite(s1_req.cmd)
        c._1 && s1_need_check && cmd_match && addr_match && mask_match
      }

      val s2_uncached_hits = RegEnable(s1_uncached_hits.asUInt, s1_valid_not_nacked)
      s2_uncached_hits.orR
    }
    val s2_valid_hit_pre_data_ecc_and_waw = s2_valid_hit_maybe_flush_pre_data_ecc_and_waw && s2_readwrite && !s2_no_alloc_hazard
    val s2_valid_flush_line = s2_valid_hit_maybe_flush_pre_data_ecc_and_waw && s2_cmd_flush_line
    val s2_valid_hit_pre_data_ecc = s2_valid_hit_pre_data_ecc_and_waw && (!s2_waw_hazard || s2_store_merge)
    val s2_valid_data_error = s2_valid_hit_pre_data_ecc_and_waw && s2_data_error
    val s2_valid_hit = s2_valid_hit_pre_data_ecc && !s2_data_error
    val s2_valid_miss = s2_valid_masked && s2_readwrite && !s2_meta_error && !s2_hit
    val s2_uncached = !s2_pma.cacheable || s2_req.no_alloc && !s2_pma.must_alloc && !s2_hit_valid // ! no_alloc never seems to be assigned, so 0 by default
    val s2_valid_cached_miss = s2_valid_miss && !s2_uncached && !uncachedInFlight.asUInt.orR
    dontTouch(s2_valid_cached_miss)
    val s2_want_victimize = Bool(!usingDataScratchpad) && (s2_valid_cached_miss || s2_valid_flush_line || s2_valid_data_error)
    val s2_valid_uncached_pending = s2_valid_miss && s2_uncached && !uncachedInFlight.asUInt.andR
    val s2_victim_way = UIntToOH(RegEnable(s1_victim_way, s1_valid_not_nacked))
    val s2_victim_or_hit_way = Mux(s2_hit_valid, s2_hit_way, s2_victim_way)
    val s2_victim_tag = Mux(s2_valid_data_error || s2_valid_flush_line, s2_req.addr(paddrBits-1, tagLSB), Mux1H(s2_victim_way, s2_meta_corrected).tag)
    val s2_victim_state = Mux(s2_hit_valid, s2_hit_state, Mux1H(s2_victim_way, s2_meta_corrected).coh)
    val s2_cannot_victimize = io.cpu.s2_kill || (!s2_victim_state.isStable())
    val s2_victimize = s2_want_victimize && !s2_cannot_victimize
    when (!s2_victim_state.isStable() && s2_want_victimize) {
      s1_nack := true
    }

    assert(s2_probe_state =/= CustomClientMetadata(CustomClientStates.IS))
    val (s2_prb_ack_data, s2_report_param, probeNewCoh)= s2_probe_state.onProbe(probe_bits.param) 
    val (s2_victim_dirty, s2_shrink_param, voluntaryNewCoh) = s2_victim_state.onCacheControl(M_FLUSH)
    dontTouch(s2_victim_dirty)
    val s2_update_meta = s2_hit_state =/= s2_new_hit_state
    val s2_dont_nack_uncached = s2_valid_uncached_pending && tl_out_a.ready
    val s2_dont_nack_misc = s2_valid_masked && !s2_meta_error && s2_req.cmd === M_WOK
    io.cpu.s2_nack := s2_valid_no_xcpt && !s2_dont_nack_uncached && !s2_dont_nack_misc && !s2_valid_hit //jamesNextTest: consider adding the "victim is unstable" case here instead of doing an s1_nack
    when (io.cpu.s2_nack || (s2_valid_hit_pre_data_ecc_and_waw && s2_update_meta)) { s1_nack := true } //if something was nacked in the s2, or if there is something being updated in metadata then we need to nack s1 to make time

    // tag updates on ECC errors
    val s2_first_meta_corrected = PriorityMux(s2_meta_correctable_errors, s2_meta_corrected)

    // load reservations and TL error reporting
    val s2_lr = Bool(usingAtomics && !usingDataScratchpad) && s2_req.cmd === M_XLR
    val s2_sc = Bool(usingAtomics && !usingDataScratchpad) && s2_req.cmd === M_XSC
    val lrscCount = Reg(init=UInt(0))
    val lrscValid = lrscCount > lrscBackoff
    val lrscBackingOff = lrscCount > 0 && !lrscValid
    val lrscAddr = Reg(UInt())
    val lrscAddrMatch = lrscAddr === (s2_req.addr >> blockOffBits)
    val s2_sc_fail = s2_sc && !(lrscValid && lrscAddrMatch)
    when ((s2_valid_hit && s2_lr && !cached_grant_wait || s2_valid_cached_miss) && !io.cpu.s2_kill) {
      lrscCount := Mux(s2_hit, lrscCycles - 1, 0.U)
      lrscAddr := s2_req.addr >> blockOffBits
    }
    when (lrscCount > 0) { lrscCount := lrscCount - 1 }
    when (s2_valid_not_killed && lrscValid) { lrscCount := lrscBackoff }
    when (s1_probe) { lrscCount := 0 }

    // don't perform data correction if it might clobber a recent store
    val s2_correct = s2_data_error && !any_pstore_valid && !RegNext(any_pstore_valid || s2_valid) && Bool(usingDataScratchpad)
    // pending store buffer

    //How in the world does this work?  I just don't understand.  This is where all the store logic happens.  It seems to run parallel to pipeline
    val s2_valid_correct = s2_valid_hit_pre_data_ecc_and_waw && s2_correct && !io.cpu.s2_kill
    def s2_store_valid_pre_kill = s2_valid_hit && s2_write && !s2_sc_fail
    def s2_store_valid = s2_store_valid_pre_kill && !io.cpu.s2_kill
    val pstore1_cmd = RegEnable(s1_req.cmd, s1_valid_not_nacked && s1_write)
    val pstore1_addr = RegEnable(s1_vaddr, s1_valid_not_nacked && s1_write)
    val pstore1_data = RegEnable(io.cpu.s1_data.data, s1_valid_not_nacked && s1_write)
    val pstore1_way = RegEnable(s1_hit_way, s1_valid_not_nacked && s1_write)
    val pstore1_mask = RegEnable(s1_mask, s1_valid_not_nacked && s1_write)
    val pstore1_storegen_data = Wire(init = pstore1_data)
    val pstore1_rmw = Bool(usingRMW) && RegEnable(needsRead(s1_req), s1_valid_not_nacked && s1_write)
    val pstore1_merge_likely = s2_valid_not_nacked_in_s1 && s2_write && s2_store_merge
    val pstore1_merge = s2_store_valid && s2_store_merge
    val pstore2_valid = RegInit(false.B)
    val pstore_drain_opportunistic = !(io.cpu.req.valid && likelyNeedsRead(io.cpu.req.bits)) && !(s1_valid && s1_waw_hazard)
    val pstore_drain_on_miss = releaseInFlight || RegNext(io.cpu.s2_nack)
    val pstore1_held = RegInit(false.B)
    val pstore1_valid_likely = s2_valid && s2_write || pstore1_held
    def pstore1_valid_not_rmw(s2_kill: Bool) = s2_valid_hit_pre_data_ecc && s2_write && !s2_kill || pstore1_held
    val pstore1_valid = s2_store_valid || pstore1_held
    any_pstore_valid := pstore1_held || pstore2_valid
    val pstore_drain_structural = pstore1_valid_likely && pstore2_valid && ((s1_valid && s1_write) || pstore1_rmw)
    assert(pstore1_rmw || pstore1_valid_not_rmw(io.cpu.s2_kill) === pstore1_valid)
    ccover(pstore_drain_structural, "STORE_STRUCTURAL_HAZARD", "D$ read-modify-write structural hazard")
    ccover(pstore1_valid && pstore_drain_on_miss, "STORE_DRAIN_ON_MISS", "D$ store buffer drain on miss")
    ccover(s1_valid_not_nacked && s1_waw_hazard, "WAW_HAZARD", "D$ write-after-write hazard")
    def should_pstore_drain(truly: Bool) = {
      val s2_kill = truly && io.cpu.s2_kill
      !pstore1_merge_likely &&
      (Bool(usingRMW) && pstore_drain_structural ||
        (((pstore1_valid_not_rmw(s2_kill) && !pstore1_rmw) || pstore2_valid) && (pstore_drain_opportunistic || pstore_drain_on_miss)))
    }
    val pstore_drain = should_pstore_drain(true)
    pstore1_held := (s2_store_valid && !s2_store_merge || pstore1_held) && pstore2_valid && !pstore_drain
    val advance_pstore1 = (pstore1_valid || s2_valid_correct) && (pstore2_valid === pstore_drain)
    pstore2_valid := pstore2_valid && !pstore_drain || advance_pstore1
    val pstore2_addr = RegEnable(Mux(s2_correct, s2_vaddr, pstore1_addr), advance_pstore1)
    val pstore2_way = RegEnable(Mux(s2_correct, s2_hit_way, pstore1_way), advance_pstore1)
    val pstore2_storegen_data = {
      for (i <- 0 until wordBytes)
        yield RegEnable(pstore1_storegen_data(8*(i+1)-1, 8*i), advance_pstore1 || pstore1_merge && pstore1_mask(i))
    }.asUInt
    val pstore2_storegen_mask = {
      val mask = Reg(UInt(width = wordBytes))
      when (advance_pstore1 || pstore1_merge) {
        val mergedMask = pstore1_mask | Mux(pstore1_merge, mask, 0.U)
        mask := ~Mux(s2_correct, 0.U, ~mergedMask)
      }
      mask
    }
    s2_store_merge := (if (eccBytes == 1) false.B else {
      ccover(pstore1_merge, "STORE_MERGED", "D$ store merged")
      // only merge stores to ECC granules that are already stored-to, to avoid
      // WAW hazards
      val wordMatch = (eccMaskFunc(pstore2_storegen_mask) | ~eccMaskFunc(pstore1_mask)).andR
      val idxMatch = s2_vaddr(untagBits-1, log2Ceil(wordBytes)) === pstore2_addr(untagBits-1, log2Ceil(wordBytes))
      val tagMatch = (s2_hit_way & pstore2_way).orR
      pstore2_valid && wordMatch && idxMatch && tagMatch
    })
 
    writeDataCPU()

    // store->load RAW hazard detection
    def s1Depends(addr: UInt, mask: UInt) =
      addr(idxMSB, wordOffBits) === s1_vaddr(idxMSB, wordOffBits) &&
      Mux(s1_write, (eccByteMask(mask) & eccByteMask(s1_mask_xwr)).orR, (mask & s1_mask_xwr).orR)
    val s1_hazard =
      (pstore1_valid_likely && s1Depends(pstore1_addr, pstore1_mask)) ||
      (pstore2_valid && s1Depends(pstore2_addr, pstore2_storegen_mask))
    val s1_raw_hazard = s1_read && s1_hazard
    s1_waw_hazard := (if (eccBytes == 1) false.B else {
      ccover(s1_valid_not_nacked && s1_waw_hazard, "WAW_HAZARD", "D$ write-after-write hazard")
      s1_write && (s1_hazard || needsRead(s1_req) && !s1_did_read)
    })
    when (s1_valid && s1_raw_hazard) { s1_nack := true }

    // performance hints to processor
    io.cpu.s2_nack_cause_raw := RegNext(s1_raw_hazard) || !(!s2_waw_hazard || s2_store_merge)

    // Prepare a TileLink request message that initiates a transaction
    val a_source = PriorityEncoder(~uncachedInFlight.asUInt << mmioOffset) // skip the MSHR
    val acquire_address = (s2_req.addr >> idxLSB) << idxLSB
    val access_address = s2_req.addr
    val a_size = s2_req.size
    val a_data = Fill(beatWords, pstore1_data)
    val a_mask = pstore1_mask << (access_address.extract(beatBytes.log2-1, wordBytes.log2) << 3)
    val get     = edge.Get(a_source, access_address, a_size)._2
    val put     = edge.Put(a_source, access_address, a_size, a_data)._2
    val putpartial = edge.Put(a_source, access_address, a_size, a_data, a_mask)._2
    val atomics = if (edge.manager.anySupportLogical) {
      MuxLookup(s2_req.cmd, Wire(new TLBundleA(edge.bundle)), Array(
        M_XA_SWAP -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.SWAP)._2,
        M_XA_XOR  -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.XOR) ._2,
        M_XA_OR   -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.OR)  ._2,
        M_XA_AND  -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.AND) ._2,
        M_XA_ADD  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.ADD)._2,
        M_XA_MIN  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MIN)._2,
        M_XA_MAX  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MAX)._2,
        M_XA_MINU -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MINU)._2,
        M_XA_MAXU -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MAXU)._2))
    } else {
      // If no managers support atomics, assert fail if processor asks for them
      assert (!(tl_out_a.valid && s2_read && s2_write && s2_uncached))
      Wire(new TLBundleA(edge.bundle))
    }

    // Set pending bits for outstanding TileLink transaction
    val a_sel = UIntToOH(a_source, maxUncachedInFlight+mmioOffset) >> mmioOffset //a_source represented as a one-hot
    when (tl_out_a.fire()) {
      when (s2_uncached) {
        (a_sel.asBools zip (uncachedInFlight zip uncachedReqs)) foreach { case (s, (f, r)) =>
          when (s) {
            f := Bool(true) // ! uncachedInFlight = is true when an uncached request is sent && a_sel.asBools
            r := s2_req
            r.cmd := Mux(s2_write, Mux(s2_req.cmd === M_PWR, M_PWR, M_XWR), M_XRD)
          }
        }
      }.otherwise {
        cached_grant_wait := true
        refill_way := s2_victim_or_hit_way
      }
    }

    // grant
    val (d_first, d_last, d_done, d_address_inc) = edge.addr_inc(tl_out.d)
    val (d_opc, grantIsUncached, grantIsUncachedData) = {
      val uncachedGrantOpcodesSansData = Seq(AccessAck, HintAck)
      val uncachedGrantOpcodesWithData = Seq(AccessAckData)
      val uncachedGrantOpcodes = uncachedGrantOpcodesWithData ++ uncachedGrantOpcodesSansData
      val whole_opc = tl_out.d.bits.opcode
      if (usingDataScratchpad) {
        assert(!tl_out.d.valid || whole_opc.isOneOf(uncachedGrantOpcodes))
        // the only valid TL-D messages are uncached, so we can do some pruning
        val opc = whole_opc(uncachedGrantOpcodes.map(_.getWidth).max - 1, 0)
        val data = DecodeLogic(opc, uncachedGrantOpcodesWithData, uncachedGrantOpcodesSansData)
        (opc, true.B, data)
      } else {
        (whole_opc, whole_opc.isOneOf(uncachedGrantOpcodes), whole_opc.isOneOf(uncachedGrantOpcodesWithData))
      }
    }
    tl_d_data_encoded := encodeData(tl_out.d.bits.data, tl_out.d.bits.corrupt && !io.ptw.customCSRs.suppressCorruptOnGrantData && !grantIsUncached)
    val grantIsCached = d_opc.isOneOf(Grant, GrantData)
    val grantIsVoluntary = d_opc === ReleaseAck // Clears a different pending bit
    val grantIsRefill = d_opc === GrantData     // Writes the data array
    val grantInProgress = Reg(init=Bool(false))
    val blockProbeAfterGrantCount = Reg(init=UInt(0))
    when (blockProbeAfterGrantCount > 0) { blockProbeAfterGrantCount := blockProbeAfterGrantCount - 1 }
    val canAcceptCachedGrant = !release_state.isOneOf(s_voluntary_writeback, s_voluntary_write_meta, s_voluntary_release, s_voluntary_ack_write_meta)
    tl_out.d.ready := Mux(grantIsCached, (!d_first || tl_out.e.ready) && canAcceptCachedGrant, release_state === s_ready/*true.B*/) //always ready for ReleaseAck, new grants are only accepted when e is ready and not evicting //jamesCurrTest
    val uncachedRespIdxOH = UIntToOH(tl_out.d.bits.source, maxUncachedInFlight+mmioOffset) >> mmioOffset
    uncachedResp := Mux1H(uncachedRespIdxOH, uncachedReqs)
    when (tl_out.d.fire()) {
      when (grantIsCached) {
        grantInProgress := true
        assert(cached_grant_wait, "A GrantData was unexpected by the dcache.")
        when(d_last) {
          cached_grant_wait := false
          grantInProgress := false
          blockProbeAfterGrantCount := blockProbeAfterGrantCycles - 1
          replacer.miss
        }
      } .elsewhen (grantIsUncached) {
        (uncachedRespIdxOH.asBools zip uncachedInFlight) foreach { case (s, f) =>
          when (s && d_last) {
            assert(f, "An AccessAck was unexpected by the dcache.") // TODO must handle Ack coming back on same cycle!
            f := false // ! if grant is received then message is no longer in flight, uncachedInFlight=false
          }
        }
        when (grantIsUncachedData) {
          if (!cacheParams.separateUncachedResp) {
            if (!cacheParams.pipelineWayMux)
              s1_data_way := 1.U << nWays
            s2_req.cmd := M_XRD
            s2_req.size := uncachedResp.size
            s2_req.signed := uncachedResp.signed
            s2_req.tag := uncachedResp.tag
            s2_req.addr := {
              require(rowOffBits >= beatOffBits)
              val dontCareBits = s1_paddr >> rowOffBits << rowOffBits
              dontCareBits | uncachedResp.addr(beatOffBits-1, 0)
            }
            s2_uncached_resp_addr := uncachedResp.addr
          }
        }
      } .elsewhen (grantIsVoluntary) {
        assert(release_ack_wait, "A ReleaseAck was unexpected by the dcache.") // TODO should handle Ack coming back on same cycle!
        //release_ack_wait := false
      }
    }

    // ! generic message types (for finishing TileLink transaction with grantAck)
    val grantAck = edge.GrantAck(tl_out.d.bits)
    val nullEMsg = edge.GrantAck(tl_out.d.bits) // should always be used with valid = 0
    assert(tl_out.e.fire() === (tl_out.d.fire() && d_first && grantIsCached))

    // data refill
    // note this ready-valid signaling ignores E-channel backpressure, which
    // benignly means the data RAM might occasionally be redundantly written
    when (grantIsRefill && !dataArb.io.in(1).ready) {
      tl_out.e.valid := false
      tl_out.d.ready := false
    }
    if (!usingDataScratchpad) {
      //writeDataGrant()
    } else {
      //jamesTODO: can be deleted
      dataArb.io.in(1).valid := tl_out.d.valid && grantIsRefill && canAcceptCachedGrant
      dataArb.io.in(1).bits := dataArb.io.in(0).bits
    }

    // Handle an incoming TileLink Probe message
    val block_probe_for_core_progress = blockProbeAfterGrantCount > 0 || lrscValid
    val block_probe_for_pending_release_ack = release_ack_wait && (tl_out.b.bits.address ^ release_ack_addr)(((pgIdxBits + pgLevelBits) min paddrBits) - 1, idxLSB) === 0
    val block_probe_for_ordering = releaseInFlight || block_probe_for_pending_release_ack || grantInProgress || s2_probe_stall

    //block b messages if we're still waiting for one to be handled, block_probe cases, or if we already have something in s1 or s2
    tl_out.b.ready := metaArb.io.in(6).ready && !(block_probe_for_core_progress || block_probe_for_ordering || s1_valid || s2_valid || s1_probe) //jamesTodo: need to test the addition of s1_probe

    // replacement policy... voodoo... I don't even know which if case gets used but I'm pretty sure it's the else case
    s1_victim_way := (if (replacer.perSet && nWays > 1) {
      val repl_array = Mem(nSets, UInt(replacer.nBits.W))
      val s1_repl_idx = s1_req.addr(idxBits+blockOffBits-1, blockOffBits)
      val s2_repl_idx = s2_vaddr(idxBits+blockOffBits-1, blockOffBits)
      val s2_repl_state = Reg(UInt(replacer.nBits.W))
      val s2_new_repl_state = replacer.get_next_state(s2_repl_state, OHToUInt(s2_hit_way))
      val s2_repl_wen = s2_valid_masked && s2_hit_way.orR && s2_repl_state =/= s2_new_repl_state
      val s1_repl_state = Mux(s2_repl_wen && s2_repl_idx === s1_repl_idx, s2_new_repl_state, repl_array(s1_repl_idx))
      when (s1_valid_not_nacked) { s2_repl_state := s1_repl_state }

      val waddr = Mux(resetting, flushCounter(idxBits-1, 0), s2_repl_idx)
      val wdata = Mux(resetting, 0.U, s2_new_repl_state)
      val wen = resetting || s2_repl_wen
      when (wen) { repl_array(waddr) := wdata }

      replacer.get_replace_way(s1_repl_state)
    } else {
      replacer.way
    })

    // release
    val (c_first, c_last, releaseDone, c_count) = edge.count(tl_out_c)
    val releaseRejected = Wire(Bool())
    val s1_release_data_valid = Reg(next = dataArb.io.in(2).fire())
    val s2_release_data_valid = RegEnable(next = s1_release_data_valid && !releaseRejected, !s2_probe_stall)
    releaseRejected := s2_release_data_valid && !tl_out_c.fire()
    val releaseDataBeat = Cat(UInt(0), c_count) + Mux(releaseRejected, UInt(0), s1_release_data_valid + Cat(UInt(0), s2_release_data_valid))

    //some generic message constructors to be re-used
    val nackResponseMessage = edge.ProbeAck(b = probe_bits, reportPermissions = TLPermissions.NtoN)
    val cleanProbeAckMessage = edge.ProbeAck(b = probe_bits, reportPermissions = s2_report_param) 
    val dirtyProbeAckMessage = edge.ProbeAck(b = probe_bits, reportPermissions = s2_report_param, data = 0.U)
    val releaseMessage = edge.Release(fromSource = 0.U, toAddress = 0.U, lgSize = lgCacheBlockBytes, shrinkPermissions = s2_shrink_param)._2
    val releaseDataMessage = edge.Release(fromSource = 0.U, toAddress = 0.U, lgSize = lgCacheBlockBytes, shrinkPermissions = s2_shrink_param, data = 0.U)._2


    // JamesTODO: CONSIDER DELETING THIS IN FAVOR OF THE OTHERWISE STATEMENT IN FSM
    val newCoh = Wire(init = probeNewCoh)
    releaseWay := s2_probe_way

    //Input Types
    val grantToT = grantIsCached && tl_out.d.valid && tl_out.d.bits.param === TLPermissions.toT
    val grantToB = grantIsCached && tl_out.d.valid && tl_out.d.bits.param === TLPermissions.toB
    val releaseAck = tl_out.d.fire() && tl_out.d.bits.opcode === 6.U //jamesCurrTest
    val CPUWrite = !io.cpu.s2_kill && isWrite(s2_req.cmd) && s2_req_valid
    val CPUWriteIntent = !io.cpu.s2_kill && isWriteIntent(s2_req.cmd) && s2_req_valid
    val CPURead =  !io.cpu.s2_kill && isRead(s2_req.cmd) && s2_req_valid

    //JamesToDid: added this test for simpler wavetrace reading
    //JamesToDo: remove this test
    val isDesiredAddr = io.cpu.req.valid && (io.cpu.req.bits.addr === "h85000000".U || io.cpu.req.bits.addr === "h86000000".U || io.cpu.req.bits.addr === "h87000000".U)
    assert(isDesiredAddr || !CPURead || !CPUWrite || !CPUWriteIntent || io.cpu.req.valid || !io.cpu.req.valid)



    //THIS IS WHERE MOST OF MY LOGIC IS CALLED
    if (!usingDataScratchpad) { // ! always enter this if statement
      newFSM()
      uncachedFSM()
      probeFSM()

      //might as well always assign these... the valid signal will sort out he details of when the message is sent
      tl_out_c.bits.source := probe_bits.source
      tl_out_c.bits.address := probe_bits.address
      tl_out_c.bits.data := s2_data_corrected
      tl_out_c.bits.corrupt := inWriteback && s2_data_error_uncorrectable

      //always send a nack message if there's nothing else to send... pretty sure this never actually gets used but who knows
      when (release_state.isOneOf(s_ready, s_dummy, s_probe_write_meta)) {
        sendCMessage(nackResponseMessage, (s2_release_data_valid || (!cacheParams.silentDrop && release_state === s_voluntary_release)))
      }

      //progress the probeFSM (I pulled these parts out and hope to pull out more)
      when (releaseDone) {
        when (release_state === s_probe_rep_miss) {
          release_state := s_ready
        } .elsewhen (release_state.isOneOf(s_probe_rep_clean, s_probe_rep_dirty)) {
          release_state := s_probe_write_meta
        } .elsewhen (release_state.isOneOf(s_voluntary_writeback, s_voluntary_write_meta, s_voluntary_release)) {
          when (!releaseAck) {
            release_state := s_voluntary_write_meta
          } .otherwise {
            release_state := s_voluntary_ack_write_meta
          }
        }
        assert(release_state =/= s_voluntary_ack_write_meta)
      }

    }

    // Drive APROT Bits (A channel)
    tl_out_a.bits.user.lift(AMBAProt).foreach { x =>
      val user_bit_cacheable = s2_pma.cacheable

      x.privileged  := s2_req.dprv === PRV.M || user_bit_cacheable
      // if the address is cacheable, enable outer caches
      x.bufferable  := user_bit_cacheable
      x.modifiable  := user_bit_cacheable
      x.readalloc   := user_bit_cacheable
      x.writealloc  := user_bit_cacheable

      // Following are always tied off
      x.fetch       := false.B
      x.secure      := true.B
    }

    tl_out_c.bits.user.lift(AMBAProt).foreach { x =>
      x.fetch       := false.B
      x.secure      := true.B
      x.privileged  := true.B
      x.bufferable  := true.B
      x.modifiable  := true.B
      x.readalloc   := true.B
      x.writealloc  := true.B
    }

    if (!cacheParams.separateUncachedResp) {
      // don't accept uncached grants if there's a structural hazard on s2_data...
      val blockUncachedGrant = Reg(Bool())
      blockUncachedGrant := dataArb.io.out.valid
      when (grantIsUncachedData && (blockUncachedGrant || s1_valid)) {
        tl_out.d.ready := false
        // ...but insert bubble to guarantee grant's eventual forward progress
        when (tl_out.d.valid) {
          io.cpu.req.ready := false
          dataArb.io.in(1).valid := true
          dataArb.io.in(1).bits.write := false
          blockUncachedGrant := !dataArb.io.in(1).ready
        }
      }
    }
    ccover(tl_out.d.valid && !tl_out.d.ready, "BLOCK_D", "D$ D-channel blocked")

    readMetaProbe()

    readDataRelease()

    writeMetaRelease()
    when (metaArb.io.in(4).fire()) { release_state := s_ready }
    //when (release_state.isOneOf(s_voluntary_write_meta, s_probe_write_meta, s_voluntary_ack_write_meta)) { release_state := s_ready }

    // cached response
    //jamesTODO: THIS IS WHERE WE RESPOND TO CPU
    io.cpu.resp.bits <> s2_req //basically return the bits it originally provided
    io.cpu.resp.bits.has_data := s2_read //it has data only if it was a read
    io.cpu.resp.bits.replay := false //no need to replay
    io.cpu.s2_uncached := s2_uncached && !s2_hit
    io.cpu.s2_paddr := s2_req.addr //update address

    // report whether there are any outstanding accesses.  disregard any
    // slave-port accesses, since they don't affect local memory ordering.
    val s1_isSlavePortAccess = s1_req.no_xcpt
    val s2_isSlavePortAccess = s2_req.no_xcpt
    io.cpu.ordered := !(s1_valid && !s1_isSlavePortAccess || s2_valid && !s2_isSlavePortAccess || cached_grant_wait || uncachedInFlight.asUInt.orR)

    val s1_xcpt_valid = tlb.io.req.valid && !s1_isSlavePortAccess && !s1_nack
    io.cpu.s2_xcpt := Mux(RegNext(s1_xcpt_valid), s2_tlb_xcpt, 0.U.asTypeOf(s2_tlb_xcpt))
    ccover(tl_out.b.valid && !tl_out.b.ready, "BLOCK_B", "D$ B-channel blocked")

    // uncached response
    val s1_uncached_data_word = {
      val word_idx = uncachedResp.addr.extract(log2Up(rowBits/8)-1, log2Up(wordBytes))
      val words = tl_out.d.bits.data.grouped(wordBits)
      words(word_idx)
    }
    val s2_uncached_data_word = RegEnable(s1_uncached_data_word, io.cpu.replay_next)
    val doUncachedResp = Reg(next = io.cpu.replay_next)
    io.cpu.resp.valid := (s2_valid_hit_pre_data_ecc || doUncachedResp) && !s2_data_error
    io.cpu.replay_next := tl_out.d.fire() && grantIsUncachedData && !cacheParams.separateUncachedResp
    when (doUncachedResp) {
      assert(!s2_valid_hit)
      io.cpu.resp.bits.replay := true
      io.cpu.resp.bits.addr := s2_uncached_resp_addr
    }

    io.cpu.uncached_resp.map { resp =>
      resp.valid := tl_out.d.valid && grantIsUncachedData
      resp.bits.tag := uncachedResp.tag
      resp.bits.size := uncachedResp.size
      resp.bits.signed := uncachedResp.signed
      resp.bits.data := new LoadGen(uncachedResp.size, uncachedResp.signed, uncachedResp.addr, s1_uncached_data_word, false.B, wordBytes).data
      resp.bits.data_raw := s1_uncached_data_word
      when (grantIsUncachedData && !resp.ready) {
        tl_out.d.ready := false
      }
    }

    // load data subword mux/sign extension
    val s2_data_word = (0 until rowBits by wordBits).map(i => s2_data_uncorrected(wordBits+i-1,i)).reduce(_|_)
    val s2_data_word_corrected = (0 until rowBits by wordBits).map(i => s2_data_corrected(wordBits+i-1,i)).reduce(_|_)
    val s2_data_word_possibly_uncached = Mux(cacheParams.pipelineWayMux && doUncachedResp, s2_uncached_data_word, 0.U) | s2_data_word
    val loadgen = new LoadGen(s2_req.size, s2_req.signed, s2_req.addr, s2_data_word_possibly_uncached, s2_sc, wordBytes)
    //lots of black magic fuckery from loadgen and pstore... please send help
    io.cpu.resp.bits.data := loadgen.data | s2_sc_fail
    io.cpu.resp.bits.data_word_bypass := loadgen.wordData
    io.cpu.resp.bits.data_raw := s2_data_word
    io.cpu.resp.bits.store_data := pstore1_data

    // AMOs
    if (usingRMW) {
      val amoalus = (0 until coreDataBits / xLen).map { i =>
        val amoalu = Module(new AMOALU(xLen))
        amoalu.io.mask := pstore1_mask >> (i * xBytes)
        amoalu.io.cmd := (if (usingAtomicsInCache) pstore1_cmd else M_XWR)
        amoalu.io.lhs := s2_data_word >> (i * xLen)
        amoalu.io.rhs := pstore1_data >> (i * xLen)
        amoalu
      }
      pstore1_storegen_data := (if (!usingDataScratchpad) amoalus.map(_.io.out).asUInt else {
        val mask = FillInterleaved(8, Mux(s2_correct, 0.U, pstore1_mask))
        amoalus.map(_.io.out_unmasked).asUInt & mask | s2_data_word_corrected & ~mask
      })
    } else if (!usingAtomics) {
      assert(!(s1_valid_masked && s1_read && s1_write), "unsupported D$ operation")
    }

    if (coreParams.useVector) {
      edge.manager.managers.foreach { m =>
        // Statically ensure that no-allocate accesses are permitted.
        // We could consider turning some of these into dynamic PMA checks.
        require(!m.supportsAcquireB || m.supportsGet, "With a vector unit, cacheable memory must support Get")
        require(!m.supportsAcquireT || m.supportsPutPartial, "With a vector unit, cacheable memory must support PutPartial")
      }
    }

    // flushes
    if (!usingDataScratchpad)
      when (RegNext(reset)) { resetting := true }
    val flushCounterNext = flushCounter +& 1
    val flushDone = (flushCounterNext >> log2Ceil(nSets)) === nWays
    val flushCounterWrap = flushCounterNext(log2Ceil(nSets)-1, 0)
    ccover(s2_valid_masked && s2_cmd_flush_all && s2_meta_error, "TAG_ECC_ERROR_DURING_FENCE_I", "D$ ECC error in tag array during cache flush")
    ccover(s2_valid_masked && s2_cmd_flush_all && s2_data_error, "DATA_ECC_ERROR_DURING_FENCE_I", "D$ ECC error in data array during cache flush")
    s1_flush_valid := false

    writeMetaReset()

    when (resetting) {
      flushCounter := flushCounterNext
      when (flushDone) {
        resetting := false
        if (!isPow2(nWays)) flushCounter := flushCounterWrap
      }
    }

    // gate the clock
    clock_en_reg := !cacheParams.clockGate ||
      io.ptw.customCSRs.disableDCacheClockGate ||
      io.cpu.keep_clock_enabled ||
      metaArb.io.out.valid || // subsumes resetting || flushing
      s1_probe || s2_probe ||
      s1_valid || s2_valid ||
      tlb_port.req.valid ||
      s1_tlb_req_valid || s2_tlb_req_valid ||
      pstore1_held || pstore2_valid ||
      release_state =/= s_ready ||
      release_ack_wait || !release_queue_empty ||
      !tlb.io.req.ready ||
      cached_grant_wait || uncachedInFlight.asUInt.orR ||
      lrscCount > 0 || blockProbeAfterGrantCount > 0

    // performance events
    io.cpu.perf.acquire := edge.done(tl_out_a)
    io.cpu.perf.release := edge.done(tl_out_c)
    io.cpu.perf.grant := tl_out.d.valid && d_last
    io.cpu.perf.tlbMiss := io.ptw.req.fire()
    io.cpu.perf.storeBufferEmptyAfterLoad := !(
      (s1_valid && s1_write) ||
      ((s2_valid && s2_write && !s2_waw_hazard) || pstore1_held) ||
      pstore2_valid)
    io.cpu.perf.storeBufferEmptyAfterStore := !(
      (s1_valid && s1_write) ||
      (s2_valid && s2_write && pstore1_rmw) ||
      ((s2_valid && s2_write && !s2_waw_hazard || pstore1_held) && pstore2_valid))
    io.cpu.perf.canAcceptStoreThenLoad := !(
      ((s2_valid && s2_write && pstore1_rmw) && (s1_valid && s1_write && !s1_waw_hazard)) ||
      (pstore2_valid && pstore1_valid_likely && (s1_valid && s1_write)))
    io.cpu.perf.canAcceptStoreThenRMW := io.cpu.perf.canAcceptStoreThenLoad && !pstore2_valid
    io.cpu.perf.canAcceptLoadThenLoad := !((s1_valid && s1_write && needsRead(s1_req)) && ((s2_valid && s2_write && !s2_waw_hazard || pstore1_held) || pstore2_valid))
    io.cpu.perf.blocked := {
      // stop reporting blocked just before unblocking to avoid overly conservative stalling
      val beatsBeforeEnd = outer.crossing match {
        case SynchronousCrossing(_) => 2
        case RationalCrossing(_) => 1 // assumes 1 < ratio <= 2; need more bookkeeping for optimal handling of >2
        case _: AsynchronousCrossing => 1 // likewise
        case _: CreditedCrossing     => 1 // likewise
      }
      val near_end_of_refill = if (cacheBlockBytes / beatBytes <= beatsBeforeEnd) tl_out.d.valid else {
        val refill_count = RegInit(0.U((cacheBlockBytes / beatBytes).log2.W))
        when (tl_out.d.fire() && grantIsRefill) { refill_count := refill_count + 1 }
        refill_count >= (cacheBlockBytes / beatBytes - beatsBeforeEnd)
      }
      cached_grant_wait && !near_end_of_refill
    }

    // report errors
    val (data_error, data_error_uncorrectable, data_error_addr) =
      if (usingDataScratchpad) (s2_valid_data_error, s2_data_error_uncorrectable, s2_req.addr) else {
        (RegNext(tl_out_c.fire() && inWriteback && s2_data_error),
          RegNext(s2_data_error_uncorrectable),
          probe_bits.address) // This is stable for a cycle after tl_out_c.fire, so don't need a register
      }
    {
      val error_addr =
        Mux(metaArb.io.in(1).valid, Cat(s2_first_meta_corrected.tag, metaArb.io.in(1).bits.addr(tagLSB-1, idxLSB)),
            data_error_addr >> idxLSB) << idxLSB
      io.errors.uncorrectable.foreach { u =>
        u.valid := metaArb.io.in(1).valid && s2_meta_error_uncorrectable || data_error && data_error_uncorrectable
        u.bits := error_addr
      }
      io.errors.correctable.foreach { c =>
        c.valid := metaArb.io.in(1).valid || data_error
        c.bits := error_addr
        io.errors.uncorrectable.foreach { u => when (u.valid) { c.valid := false } }
      }
      io.errors.bus.valid := tl_out.d.fire() && (tl_out.d.bits.denied || tl_out.d.bits.corrupt)
      io.errors.bus.bits := Mux(grantIsCached, s2_req.addr >> idxLSB << idxLSB, 0.U)

      ccoverNotScratchpad(io.errors.bus.valid && grantIsCached, "D_ERROR_CACHED", "D$ D-channel error, cached")
      ccover(io.errors.bus.valid && !grantIsCached, "D_ERROR_UNCACHED", "D$ D-channel error, uncached")
    }

    if (usingDataScratchpad) {
      val data_error_cover = Seq(
        CoverBoolean(!data_error, Seq("no_data_error")),
        CoverBoolean(data_error && !data_error_uncorrectable, Seq("data_correctable_error")),
        CoverBoolean(data_error && data_error_uncorrectable, Seq("data_uncorrectable_error")))
      val request_source = Seq(
        CoverBoolean(s2_isSlavePortAccess, Seq("from_TL")),
        CoverBoolean(!s2_isSlavePortAccess, Seq("from_CPU")))

      cover(new CrossProperty(
        Seq(data_error_cover, request_source),
        Seq(),
        "MemorySystem;;Scratchpad Memory Bit Flip Cross Covers"))
    } else {

      val data_error_type = Seq(
        CoverBoolean(!s2_valid_data_error, Seq("no_data_error")),
        CoverBoolean(s2_valid_data_error && !s2_data_error_uncorrectable, Seq("data_correctable_error")),
        CoverBoolean(s2_valid_data_error && s2_data_error_uncorrectable, Seq("data_uncorrectable_error")))
      val data_error_dirty = Seq(
        CoverBoolean(!s2_victim_dirty, Seq("data_clean")),
        CoverBoolean(s2_victim_dirty, Seq("data_dirty")))
      val request_source = if (supports_flush) {
          Seq(
            CoverBoolean(!flushing, Seq("access")),
            CoverBoolean(flushing, Seq("during_flush")))
        } else {
          Seq(CoverBoolean(true.B, Seq("never_flush")))
        }
      val tag_error_cover = Seq(
        CoverBoolean( !s2_meta_error, Seq("no_tag_error")),
        CoverBoolean( s2_meta_error && !s2_meta_error_uncorrectable, Seq("tag_correctable_error")),
        CoverBoolean( s2_meta_error && s2_meta_error_uncorrectable, Seq("tag_uncorrectable_error")))
      cover(new CrossProperty(
        Seq(data_error_type, data_error_dirty, request_source, tag_error_cover),
        Seq(),
        "MemorySystem;;Cache Memory Bit Flip Cross Covers"))
    }

    ///////////////////////////////
    // ! BEGIN MODIFICATIONS HERE//
    //////////////////////////////

    def uncachedFSM() = { //Don't really care about this... this is for uncached transactions (DMA from external device)
      when (s2_uncached) {
        when (!s2_write) {
          sendAMessage(get, isAValidUncached())
        } .elsewhen (s2_req.cmd === M_PWR) {
          sendAMessage(putpartial, isAValidUncached())
        } .elsewhen (!s2_read) {
          sendAMessage(put, isAValidUncached())
        } .otherwise {
          sendAMessage(atomics, isAValidUncached())
        }
      }
    }
    
    def probeFSM() = { //this handles the secondary states and/or multi-cycle states in the probeFSM (primary states assigned in newFSM)
       when (release_state === s_probe_rep_dirty) { //writeback dirty data upon Probe, could be multicycle
        sendCMessage(dirtyProbeAckMessage, s2_release_data_valid)
      } .elsewhen (release_state.isOneOf(s_voluntary_writeback, s_voluntary_write_meta, s_voluntary_release, s_voluntary_ack_write_meta)) { // voluntary eviction
        when (release_state === s_voluntary_release) { //no data is needed... could probably handle this in a single cycle so it might need to be here
          sendCMessage(releaseMessage, (s2_release_data_valid || (!cacheParams.silentDrop && release_state === s_voluntary_release)) && !(c_first && release_ack_wait))
        }.otherwise { // writing back data upon voluntary eviction, could be multicycle
          sendCMessage(releaseDataMessage, (s2_release_data_valid || (!cacheParams.silentDrop && release_state === s_voluntary_release)) && !(c_first && release_ack_wait))
        }

        when (release_state === s_voluntary_ack_write_meta) {
          newCoh := CustomClientMetadata(CustomClientStates.I)
        } .otherwise {
          newCoh := voluntaryNewCoh //update metadata as a result of voluntary release
        }
        releaseWay := s2_victim_or_hit_way //keep track of the way referenced in the release
        when (tl_out_c.fire() && c_first) {
          release_ack_wait := true // system must stall until releaseAck is received (starting as soon as release is sent)
          release_ack_addr := probe_bits.address //keep track of the address so that other blocks can be accessed in some cases
        }
      }
    }
    
    def isAValidUncached() = { //moved logic out of uncachedFSM... basically just the valid condition for A messages
      !io.cpu.s2_kill && (s2_valid_uncached_pending || (s2_valid_cached_miss && !s2_victim_dirty)) //jamesToDo: can remove the valid_cached_miss bit
    }

    def sendAMessage(messageToSend: TLBundleA, valid: Bool) = { //sends a message along the A channel
      tl_out_a.valid := valid
      tl_out_a.bits := messageToSend
      when (!s2_uncached) {s2_hit := false} //if you need to send an A message, you must have missed
    }

    def sendCMessage(messageToSend: TLBundleC, valid: Bool) = { //sends a message along the C channel
      tl_out_c.valid := valid
      tl_out_c.bits := messageToSend
    }

    def sendEMessage(messageToSend: TLBundleE, valid: Bool) = { //sends a message along the E channel
      tl_out_e.valid := valid
      tl_out_e.bits := messageToSend
    }

    assert(s2_new_hit_state.isStable() || s2_req.addr =/= 256) //known uncached address that shows up in code

    def newFSM() = {
      tl_out_e.valid := false //update to false each cycle, then assign true where necessary (in sendEMessage)
      s2_hit := io.cpu.req.valid && !io.cpu.s2_kill || ((CPURead || (!io.cpu.req.valid)) && s2_hit_state.isValid()) //easier to hard-code this like this than within the FSM since it's not in ProtoGen's logic

      //Eviction
      when (s2_victimize) { //evicting someone as a result of a cache miss (or error or flush)
        assert(s2_victim_state.isStable())
        assert(s2_valid_flush_line || io.cpu.s2_nack) //assertion was here before me... 
        val discard_line = s2_valid_flush_line && s2_req.size(1) //temp variable... unsure of exact purpose but necessary

        when (s2_victim_dirty && !discard_line) { //if it's dirty
          release_state := s_voluntary_writeback //then we need to write the data back
        } .elsewhen (!cacheParams.silentDrop && !release_ack_wait && release_queue_empty && s2_victim_state.isValid() && (s2_valid_flush_line || s2_readwrite && !s2_hit_valid)) {
          release_state := s_voluntary_release //if it's not dirty and we can't silentDrop then we send Release to L2
        } .otherwise {
          //release_state := s_voluntary_write_meta
          release_state := s_voluntary_ack_write_meta //otherwise we just update the metadata to reflect the eviction
        }
        probe_bits := addressToProbe(s2_vaddr, Cat(s2_victim_tag, s2_req.addr(tagLSB-1, idxLSB)) << idxLSB) //update probe_bits with new eviction info
      }

      //D Channel Inputs
      when (grantIsCached && tl_out.d.valid) { //grant providing write permission
        //assert(!s2_hit_state.isStable()) Although the metadata will hold that value, grants don't trigger a metadata read
        sendEMessage(grantAck, d_first) //send a grantAck on the first cycle of the message
        writeMetaGrant(s2_hit_state.onGrant(tl_out.d.bits.param)) //update metadata to E state
        writeDataGrant() //update data with new data
      }
      when (releaseAck) {
        release_state := s_voluntary_ack_write_meta
        //may need to check if probe bits and stuff have changed to new ways, etc
        release_ack_wait := false //stop blocking transactions now that the Release is confirmed/finalized
      }

      //B Channel Inputs
      when (s2_probe) { //when probeBlock reaches s2
        assert(s2_probe_state.isStable())
        s1_nack := true //nacks the instruction to buy more time if we need to writeback, this is undone later in code when applicable
        when (s2_prb_ack_data) { // jamesToDo: make isDirty
          assert(s2_probe_state.isDirty())
          sendCMessage(dirtyProbeAckMessage, (s2_release_data_valid)) //M will conflict with any probe and needs to be written back
          release_state := s_probe_rep_dirty //this allows FSM to continue writing a multi-cycle message
        } .elsewhen (s2_probe_state.isValid()) { //jamesToDo: I think this is correct for stable states too, but not sure
          assert(!s2_probe_state.isDirty())
          sendCMessage(cleanProbeAckMessage, true.B) //E and S have no data to be written back, unclear whether metadata needs to be modified yet
          release_state := Mux(releaseDone, s_probe_write_meta, s_probe_rep_clean) //finish sending the reply and then change to s_probe_write_meta to determine what metadata changes need to occur
        } .elsewhen (!s2_probe_state.isValid()) { //redundant if statement... could be else
          s1_nack := !releaseDone  //no need to nack if we finish right away!
          sendCMessage(nackResponseMessage, true.B) //I never conflicts
          release_state := Mux(releaseDone, s_ready, s_probe_rep_miss) //finish sending message and then we can move forward
        }
      } 

      //CPU Inputs
      val (newState, newPerm) = s2_hit_state.onMiss(s2_req.cmd)

      when (CPUWrite && !s2_uncached) {
        when (!s2_hit_state.isValid()) {
          assert(s2_hit_state === CustomClientMetadata(CustomClientStates.I))
          assert(newPerm === TLPermissions.NtoT)
          sendAMessage(acquire(s2_req.addr, newPerm), !io.cpu.s2_kill && !s2_victim_dirty && !(release_ack_wait && (s2_req.addr ^ release_ack_addr)(((pgIdxBits + pgLevelBits) min paddrBits) - 1, idxLSB) === 0) && s2_valid_cached_miss) //miss, I->E
          s2_new_hit_state := newState //IM
          assert(s2_new_hit_state === CustomClientMetadata(CustomClientStates.IM))
        } .elsewhen (s2_hit_state.hasWritePermission()._2) {
          s1_nack := true 
        } .elsewhen (!s2_hit_state.hasWritePermission()._1) { //is valid but doesn't have permissions
          assert(s2_hit_state === CustomClientMetadata(CustomClientStates.S))
          when (s2_hit_state.hasReadPermission()._1) { //jamesToDo: remove this
            assert(s2_hit_state === CustomClientMetadata(CustomClientStates.S))
            s2_new_hit_state := newState //SM
            assert(newPerm === TLPermissions.BtoT)
            sendAMessage(acquire(s2_req.addr, newPerm), !io.cpu.s2_kill && !s2_victim_dirty && !(release_ack_wait && (s2_req.addr ^ release_ack_addr)(((pgIdxBits + pgLevelBits) min paddrBits) - 1, idxLSB) === 0) && s2_valid_cached_miss)
            assert(s2_new_hit_state === CustomClientMetadata(CustomClientStates.SM))
          } .otherwise {
            //jamesToDo: verify that this doesn't happen
            sendAMessage(acquire(s2_req.addr, TLPermissions.NtoT), !io.cpu.s2_kill && !s2_victim_dirty && !(release_ack_wait && (s2_req.addr ^ release_ack_addr)(((pgIdxBits + pgLevelBits) min paddrBits) - 1, idxLSB) === 0) && s2_valid_cached_miss) //miss S->E
          }
          
        } .otherwise {  //already has permissions, update state if necessary
          //jamesToDo: what about hitting on O?  notifyL2
          assert(s2_hit_state === CustomClientMetadata(CustomClientStates.M) || s2_hit_state === CustomClientMetadata(CustomClientStates.E))
          s2_new_hit_state := s2_hit_state.onWrite()
          assert(s2_new_hit_state === CustomClientMetadata(CustomClientStates.M))
          writeMetaHit(s2_new_hit_state) //jamesToDo: remove this line, it's unnecessary (I hope?)
        }
      } .elsewhen (CPUWriteIntent && !s2_uncached) { //prefetching write permissions early
        when (!s2_hit_state.isValid()) {
          assert(s2_hit_state === CustomClientMetadata(CustomClientStates.I))
          assert(newPerm === TLPermissions.NtoT)
          sendAMessage(acquire(s2_req.addr, newPerm), !io.cpu.s2_kill && !s2_victim_dirty && !(release_ack_wait && (s2_req.addr ^ release_ack_addr)(((pgIdxBits + pgLevelBits) min paddrBits) - 1, idxLSB) === 0) && s2_valid_cached_miss) //miss I->E
          s2_new_hit_state := newState
          assert(s2_new_hit_state === CustomClientMetadata(CustomClientStates.IM))
        }.elsewhen (s2_hit_state.hasWritePermission()._2) {
          s1_nack := true
        }.elsewhen (!s2_hit_state.hasWritePermission()._1) {
          assert(s2_hit_state === CustomClientMetadata(CustomClientStates.S))
          assert(newPerm === TLPermissions.BtoT)
          sendAMessage(acquire(s2_req.addr, newPerm), !io.cpu.s2_kill && !s2_victim_dirty && !(release_ack_wait && (s2_req.addr ^ release_ack_addr)(((pgIdxBits + pgLevelBits) min paddrBits) - 1, idxLSB) === 0) && s2_valid_cached_miss) //miss S->E
          s2_new_hit_state := newState
          assert(s2_new_hit_state === CustomClientMetadata(CustomClientStates.SM))
        } .otherwise {
          s2_new_hit_state := s2_hit_state
        }
      } .elsewhen (CPURead && !s2_uncached) {
        assert(!isWrite(s2_req.cmd) && !isWriteIntent(s2_req.cmd))
        when (s2_hit_state.hasReadPermission()._2) {
          s1_nack := true
        } .elsewhen (!s2_hit_state.hasReadPermission()._1) { //We only need to send a message if we miss... misses on Reads are only I
          assert(s2_hit_state === CustomClientMetadata(CustomClientStates.I))
          when (!cached_grant_wait) { //jamesToDo: is this if necessary?
          assert(newPerm === TLPermissions.NtoB)
            sendAMessage(acquire(s2_req.addr, newPerm), !io.cpu.s2_kill && !s2_victim_dirty && !(release_ack_wait && (s2_req.addr ^ release_ack_addr)(((pgIdxBits + pgLevelBits) min paddrBits) - 1, idxLSB) === 0) && s2_valid_cached_miss) //jamesToDo what about MI protocol?
            s2_new_hit_state := newState
            assert(s2_new_hit_state === CustomClientMetadata(CustomClientStates.IS))
          } .otherwise {
            s1_nack := true
          }
        } .otherwise {
          s2_new_hit_state := s2_hit_state
          //jamesToDo: should have option to update state
          //jamesToDo: notifyL2?
        }
      }

    }

    when (s2_uncached) {
      s2_new_hit_state := CustomClientMetadata(CustomClientStates.I)
    }

    def writeDataCPU() = { //where the pstore system actually writes
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
    def writeDataGrant() = { //writing data from grants
      val valid = tl_out.d.valid && grantIsRefill && canAcceptCachedGrant
      val write = true
      val addr =  (s2_vaddr >> idxLSB) << idxLSB | d_address_inc
      val way = refill_way
      val data = tl_d_data_encoded
      val wordMask = ~UInt(0, rowBytes / subWordBytes)
      val eccMask = ~UInt(0, wordBytes / eccBytes)
      writeDataHelper(addr, way, data, wordMask, eccMask, write, valid, 1.U)
    }
    def readDataRelease() = { //Reads out the data that needs to be sent in release message
      val valid = inWriteback && releaseDataBeat < refillCycles
      val data = dataArb.io.in(1).bits.wdata
      val write = false
      val addr = (probeIdx(probe_bits) << blockOffBits) | (releaseDataBeat(log2Up(refillCycles)-1,0) << rowOffBits)
      val wordMask = ~UInt(0, rowBytes / subWordBytes)
      val eccMask = ~UInt(0, wordBytes / eccBytes)
      val way = ~UInt(0, nWays)
      writeDataHelper(addr, way, data, wordMask, eccMask, write, valid, 2.U)
    }
    def readDataCPU() = { //reads the data requested by CPU
      val valid = io.cpu.req.valid && likelyNeedsRead(io.cpu.req.bits)
      val data = dataArb.io.in(1).bits.wdata
      val write = false
      val addr = io.cpu.req.bits.addr //Cat(io.cpu.req.bits.idx.getOrElse(io.cpu.req.bits.addr) >> tagLSB, io.cpu.req.bits.addr(tagLSB-1, 0))
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
      //takes the information from the ReadData/WriteData functions and funnels it into the arbiter
      dataArb.io.in(priority).valid := valid
      dataArb.io.in(priority).bits.write := write
      dataArb.io.in(priority).bits.addr := addr
      dataArb.io.in(priority).bits.way_en := way
      dataArb.io.in(priority).bits.wdata := data
      dataArb.io.in(priority).bits.wordMask := wordMask
      dataArb.io.in(priority).bits.eccMask := eccMask
    }

    def writeMetaReset() = { ///only happens when reset signal goes high.  don't mess with it
      val valid = resetting
      val write = true
      val index = flushCounter(idxBits-1, 0)
      val addr = Cat(io.cpu.req.bits.addr >> untagBits, flushCounter(idxBits-1, 0) << blockOffBits)
      val way = ~UInt(0, nWays)
      val data = tECC.encode(CustomL1Metadata(0.U, CustomClientMetadata.onReset).asUInt)
      val wmask = if (nWays == 1) Seq(true.B) else way.asBools
      writeMetaHelper(addr, way, index, data, write, valid, 0.U)
    }
    def writeMetaHit(newState: CustomClientMetadata) = { //Misnomer, also used on misses
      val valid = s2_valid_hit_pre_data_ecc_and_waw && s2_update_meta || (!s2_victim_dirty && s2_valid_cached_miss)
      val write = !io.cpu.s2_kill
      val way = Wire(Bits(width = nWays))
      way := s2_victim_or_hit_way //jamesToDo: is this ok?
      val index = s2_vaddr(idxMSB, idxLSB)
      val addr = Cat(io.cpu.req.bits.addr >> untagBits, s2_vaddr(idxMSB, 0)) //JAMESTODO: USE s2_req.addr
      assert(newState =/= CustomClientMetadata(CustomClientStates.I))
      val data = tECC.encode(CustomL1Metadata(s2_req.addr >> tagLSB, newState).asUInt)
      val wmask = if (nWays == 1) Seq(true.B) else way.asBools
      writeMetaHelper(addr, way, index, data, write, valid, 2.U)
    }
    def writeMetaGrant(newState: CustomClientMetadata) = { //updating metadata upon receiving new permissions
      val valid = grantIsCached && d_done && !tl_out.d.bits.denied
      val write = true
      val way = Wire(Bits(width = nWays))
      way := refill_way
      val index = s2_vaddr(idxMSB, idxLSB)//lastCPUAddr(idxMSB, idxLSB)//tl_out.d.bits.address(idxMSB, idxLSB) jamesToDid
      val addr = /*lastCPUAddr*/Cat(io.cpu.req.bits.addr >> untagBits, s2_vaddr(idxMSB, 0)) //tl_out.d.bits.address jamesToDid
      val data = tECC.encode(CustomL1Metadata(s2_req.addr >> tagLSB, newState).asUInt)
      val wmask = if (nWays == 1) Seq(true.B) else way.asBools
      writeMetaHelper(addr, way, index, data, write, valid, 3.U)
    }
    def writeMetaRelease() = { //downgrading permissions on voluntary and involuntary releases
      val valid = release_state.isOneOf(s_voluntary_write_meta, s_probe_write_meta, s_voluntary_ack_write_meta)
      val write = true
      probeWay := releaseWay
      val index = probeIdx(probe_bits)
      val addr = Cat(io.cpu.req.bits.addr >> untagBits, probe_bits.address(idxMSB, 0))
      probeData := tECC.encode(CustomL1Metadata(tl_out_c.bits.address >> tagLSB, newCoh).asUInt)
      val wmask = if (nWays ==1) Seq(true.B) else probeWay.asBools
      writeMetaHelper(addr, probeWay, index, probeData, write, valid, 4.U)
    }
    def readMetaProbe() = { //reading metadata to determine what action is necessary for probe
      val valid = (tl_out.b.valid && (!block_probe_for_core_progress || lrscBackingOff))
      val write = false
      val index = probeIdx(tl_out.b.bits)
      val addr = Cat(io.cpu.req.bits.addr >> paddrBits, tl_out.b.bits.address)
      val way = probeWay
      val data = probeData
      writeMetaHelper(addr, way, index, data, write, valid, 6.U)
    }
    def readMetaCPU() = { //reading metadata for typical CPU input
      val valid = io.cpu.req.valid //jamesTODO: shouldn't this be delayed by a cycle?  Am I an idiot or something?  Confusion
      val write = false
      val index = dataArb.io.in(3).bits.addr(idxMSB, idxLSB)
      val addr = io.cpu.req.bits.addr
      val way = probeWay
      val data = probeData
      writeMetaHelper(addr, way, index, data, write, valid, 7.U)
    }
    def writeMetaHelper(addr: UInt, way: UInt, index: UInt, data: UInt, write: Bool, valid: Bool, priority: UInt) = {
      //takes the information from the ReadMeta/WriteMeta functions and funnels it into the arbiter
      metaArb.io.in(priority).valid := valid
      metaArb.io.in(priority).bits.write := write
      metaArb.io.in(priority).bits.way_en := way
      metaArb.io.in(priority).bits.idx := index
      metaArb.io.in(priority).bits.addr := addr
      metaArb.io.in(priority).bits.data := data
    }

    def readMeta(addr: UInt) = {
      //yet to be implemented metadata reader... was going to use for the single-stage version... multi-stage version is probably required to use arbiter so I don't think this will ever actually be used
      val idx = addr(idxMSB, idxLSB) //pull index out of requested address
      val tag = addr >> tagLSB //pull tag out of requested address
      val allWays = tag_array.read(idx, true) //read out all ways for this set
      val allWays_uncorrected = allWays.map(tECC.decode(_).uncorrected.asTypeOf(new CustomL1Metadata)) //encoding... voodoo
      //val hit_way = allWays_uncorrected.map(r => r.coh.isValid() && r.tag === tag).asUInt
      val hit_state = CustomClientMetadata.onReset.fromBits(
        allWays_uncorrected.map(r => Mux(r.tag === tag, r.coh.asUInt, UInt(0)))
        .reduce (_|_)) //basically pick out the one way that you actually want by comparing tags
      hit_state //return hit_state
    }

  } // leaving gated-clock domain
  /////////////////////////////
  // ! END MODIFICATIONS HERE//
  /////////////////////////////





  val dcacheImpl = withClock (gated_clock) { new DCacheModuleImpl }

  def encodeData(x: UInt, poison: Bool) = x.grouped(eccBits).map(dECC.encode(_, if (dECC.canDetect) poison else false.B)).asUInt
  def dummyEncodeData(x: UInt) = x.grouped(eccBits).map(dECC.swizzle(_)).asUInt
  def decodeData(x: UInt) = x.grouped(dECC.width(eccBits)).map(dECC.decode(_))
  def eccMaskFunc(byteMask: UInt) = byteMask.grouped(eccBytes).map(_.orR).asUInt
  def eccByteMask(byteMask: UInt) = FillInterleaved(eccBytes, eccMaskFunc(byteMask))

  def likelyNeedsRead(req: HellaCacheReq) = {
    // ! if it's not an intStore or prefetch with intent to write
    // ! if it's not a store then it likely needs a read... duh?
    val res = !req.cmd.isOneOf(M_XWR, M_PFW) || req.size < log2Ceil(eccBytes)
    assert(!needsRead(req) || res)
    res
  }
  def needsRead(req: HellaCacheReq) =
    // ! determines if command is a read or if it's a partial write (which involves reading)
    isRead(req.cmd) ||
    (isWrite(req.cmd) && (req.cmd === M_PWR || req.size < log2Ceil(eccBytes)))

  def ccover(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    cover(cond, s"DCACHE_$label", "MemorySystem;;" + desc)
  def ccoverNotScratchpad(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    if (!usingDataScratchpad) ccover(cond, label, desc)

  require(!usingVM || tagLSB <= pgIdxBits, s"D$$ set size must not exceed ${1<<(pgIdxBits-10)} KiB; got ${(nSets * cacheBlockBytes)>>10} KiB")
  def tagLSB: Int = untagBits
  def probeIdx(b: TLBundleB): UInt = b.address(idxMSB, idxLSB)
  def addressToProbe(vaddr: UInt, paddr: UInt): TLBundleB = {
    val res = Wire(new TLBundleB(edge.bundle))
    res.address := paddr
    res.source := mmioOffset - 1
    res
  }
  def acquire(paddr: UInt, param: UInt): TLBundleA = {
    // ? Seems to check if AcquireB is supported by any slaves, then adjusts the message accordingly?
    if (!edge.manager.anySupportAcquireB) Wire(new TLBundleA(edge.bundle))
    else edge.AcquireBlock(UInt(0), paddr >> lgCacheBlockBytes << lgCacheBlockBytes, lgCacheBlockBytes, param)._2
  }

}