void CANwrite(int datal, int datah, int buf_id) {
    while (MDR_CAN1->BUF_CON[buf_id] & (1 << CAN_BUF_CON_TX_REQ_Pos));
    MDR_CAN1->CAN_BUF[buf_id].DATAL = datal;
    MDR_CAN1->CAN_BUF[buf_id].DATAH = datah;
    MDR_CAN1->BUF_CON[buf_id] |= 1 << CAN_BUF_CON_TX_REQ_Pos;
}