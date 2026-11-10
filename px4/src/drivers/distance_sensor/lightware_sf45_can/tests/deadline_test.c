#include <canard.h>
#include <assert.h>
#include <stdint.h>

int main(void)
{
    CanardInstance node;
    uint64_t arena[256];
    canardInit(&node, arena, sizeof(arena), 0, 0, 0);
    canardSetLocalNodeID(&node, 42);
    uint8_t tid = 0;
    const uint8_t payload[8] = {0};
    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeBroadcast,
        .data_type_signature = 0x99DD3985FB3222CEULL,
        .data_type_id = 21910,
        .inout_transfer_id = &tid,
        .priority = CANARD_TRANSFER_PRIORITY_LOW,
        .payload = payload,
        .payload_len = sizeof(payload),
        .deadline_usec = 20000,
    };
    assert(canardBroadcastObj(&node, &transfer) == 2);
    assert(canardPeekTxQueue(&node)->deadline_usec == 20000);
    canardPopTxQueue(&node);
    assert(canardPeekTxQueue(&node)->deadline_usec == 20000);
    canardCleanupStaleTransfers(&node, 20001);
    assert(canardPeekTxQueue(&node) == 0);
    return 0;
}
