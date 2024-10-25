package dongjiang.chi

import chisel3._
import chisel3.util._

object RespErr {
    val width = 2

    val NormalOkay    = "b00".U(width.W)
    val ExclusiveOkay = "b01".U(width.W)
    val DataError     = "b10".U(width.W)
    val NonDataError  = "b11".U(width.W)
}

object Order {
    val width = 2

    val None            = "b00".U(width.W)
    val RequestAccepted = "b01".U(width.W)
    val RequestOrder    = "b10".U(width.W)
    val OWO             = "b10".U(width.W) // Ordered Write Observation
    val EndpointOrder   = "b11".U(width.W)

    def isRequestOrder(order: UInt): Bool = order >= RequestOrder
}

class MemAttr extends Bundle {
    // The Allocate attribute is a an allocation hint.
    // It indicates the recommended allocation policy for a transaction.
    val allocate = Bool()
    // The Cacheable attribute indicates if a transaction must perform a cache lookup.
    val cacheable = Bool()
    // Device attribute indicates if the memory type is either Device or Normal.
    val device = Bool()
    // Early Write Acknowledge (EWA)
    // EWA indicates whether the write completion response for a transaction:
    // If true, comp is permitted to come from an intermediate point in the interconnect, such as a Home Node.
    // If false, comp must come from the final endpoint that a transaction is destined for.
    val ewa = Bool()
}

object MemAttr {
    def apply(allocate: Bool, cacheable: Bool, device: Bool, ewa: Bool): MemAttr = {
        val memAttr = Wire(new MemAttr)
        memAttr.allocate  := allocate
        memAttr.cacheable := cacheable
        memAttr.device    := device
        memAttr.ewa       := ewa
        memAttr
    }
    def apply(): MemAttr = apply(false.B, false.B, false.B, false.B)
}
