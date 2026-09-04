// See LICENSE.SiFive for license details.

package freechips.rocketchip.amba.axi4

import chisel3._
import chisel3.util.{log2Ceil, Cat}

import org.chipsalliance.cde.config.Parameters

import org.chipsalliance.diplomacy.lazymodule.{LazyModule, LazyModuleImp}

import freechips.rocketchip.diplomacy.IdRange
import freechips.rocketchip.util.{ControlKey, SimpleBundleField}

// Keyed per AXI4IdIndexer instance (via `uid`) rather than a single shared object: two indexer instances can
// legitimately appear in series on the same path (e.g. one bounding a crossbar's per-ID tracker close to its
// source, another compacting a merged ID space further downstream), and each must echo its own extra-ID bits
// independently. A shared key would make the second instance's BundleMap collide with the first's (duplicate
// field name) whenever both indexers' fields survive onto the same edge.
case class AXI4ExtraId(uid: Int) extends ControlKey[UInt](s"extra_id_$uid")
case class AXI4ExtraIdField(uid: Int, width: Int) extends SimpleBundleField(AXI4ExtraId(uid))(Output(UInt(width.W)), 0.U)

/** This adapter limits the set of FIFO domain ids used by outbound transactions.
  *
  * Extra AWID and ARID bits from upstream transactions are stored in a User Bits field called AXI4ExtraId,
  * which values are expected to be echoed back to this adapter alongside any downstream response messages,
  * and are then prepended to the RID and BID field to restore the original identifier.
  *
  * @param idBits is the desired number of A[W|R]ID bits to be used
  */
class AXI4IdIndexer(idBits: Int)(implicit p: Parameters) extends LazyModule
{
  require (idBits >= 0, s"AXI4IdIndexer: idBits must be > 0, not $idBits")

  // Unique per instance so this indexer's echo field never collides with another AXI4IdIndexer's field on a
  // shared downstream edge (see AXI4ExtraId above).
  private val uid = AXI4IdIndexer.nextUid()

  val node = AXI4AdapterNode(
    masterFn = { mp =>
      // Create one new "master" per ID
      val masters = Array.tabulate(1 << idBits) { i => AXI4MasterParameters(
         name      = "",
         id        = IdRange(i, i+1),
         aligned   = true,
         maxFlight = Some(0))
      }
      // Accumulate the names of masters we squish
      val names = Array.fill(1 << idBits) { new scala.collection.mutable.HashSet[String]() }
      // Squash the information from original masters into new ID masters
      mp.masters.foreach { m =>
        for (i <- m.id.start until m.id.end) {
          val j = i % (1 << idBits)
          val accumulated = masters(j)
          names(j) += m.name
          masters(j) = accumulated.copy(
            aligned   = accumulated.aligned && m.aligned,
            maxFlight = accumulated.maxFlight.flatMap { o => m.maxFlight.map { n => o+n } })
        }
      }
      val finalNameStrings = names.map { n => if (n.isEmpty) "(unused)" else n.toList.mkString(", ") }
      val bits = log2Ceil(mp.endId) - idBits
      val field = if (bits > 0) Seq(AXI4ExtraIdField(uid, bits)) else Nil
      mp.copy(
        echoFields = field ++ mp.echoFields,
        masters    = masters.zip(finalNameStrings).map { case (m, n) => m.copy(name = n) }.toIndexedSeq)
    },
    slaveFn = { sp => sp
    })

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>

      // Leave everything mostly untouched

      Connectable.waiveUnmatched(out.ar, in.ar) match {
        case (lhs, rhs) => lhs.squeezeAll :<>= rhs.squeezeAll
      }
      Connectable.waiveUnmatched(out.aw, in.aw) match {
        case (lhs, rhs) => lhs.squeezeAll :<>= rhs.squeezeAll
      }
      Connectable.waiveUnmatched(out.w, in.w) match {
        case (lhs, rhs) => lhs.squeezeAll :<>= rhs.squeezeAll
      }
      Connectable.waiveUnmatched(in.b, out.b) match {
        case (lhs, rhs) => lhs.squeezeAll :<>= rhs.squeezeAll
      }
      Connectable.waiveUnmatched(in.r, out.r) match {
        case (lhs, rhs) => lhs.squeezeAll :<>= rhs.squeezeAll
      }

      val bits = log2Ceil(edgeIn.master.endId) - idBits
      if (bits > 0) {
        // (in.aX.bits.id >> idBits).width = bits > 0
        out.ar.bits.echo(AXI4ExtraId(uid)) := in.ar.bits.id >> idBits
        out.aw.bits.echo(AXI4ExtraId(uid)) := in.aw.bits.id >> idBits
        // Special care is needed in case of 0 idBits, b/c .id has width 1 still
        if (idBits == 0) {
          out.ar.bits.id := 0.U
          out.aw.bits.id := 0.U
          in.r.bits.id := out.r.bits.echo(AXI4ExtraId(uid))
          in.b.bits.id := out.b.bits.echo(AXI4ExtraId(uid))
        } else {
          in.r.bits.id := Cat(out.r.bits.echo(AXI4ExtraId(uid)), out.r.bits.id)
          in.b.bits.id := Cat(out.b.bits.echo(AXI4ExtraId(uid)), out.b.bits.id)
        }
      }
    }
  }
}

object AXI4IdIndexer
{
  private var uidCounter = 0
  private[axi4] def nextUid(): Int = { val u = uidCounter; uidCounter += 1; u }

  def apply(idBits: Int)(implicit p: Parameters): AXI4Node =
  {
    val axi4index = LazyModule(new AXI4IdIndexer(idBits))
    axi4index.node
  }
}
