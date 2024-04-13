use usbd_storage::subclass::scsi::ScsiCommand;

const UNKNOWN: u8 = 0xFF;
const TEST_UNIT_READY: u8 = 0x00;
const REQUEST_SENSE: u8 = 0x03;
const INQUIRY: u8 = 0x12;
const MODE_SENSE_6: u8 = 0x1A;
const MODE_SENSE_10: u8 = 0x5A;
const READ_10: u8 = 0x28;
const READ_CAPACITY_10: u8 = 0x25;
const READ_CAPACITY_16: u8 = 0x9E;
const WRITE_10: u8 = 0x2A;
const READ_FORMAT_CAPACITIES: u8 = 0x23;

pub fn cmd_into_bytes(cmd: ScsiCommand) -> Vec<u8> {
    let mut bytes = vec![];
    match cmd {
        ScsiCommand::Unknown => {
            bytes.push(UNKNOWN);
        }
        ScsiCommand::Inquiry {
            evpd,
            page_code,
            alloc_len,
        } => {
            bytes.push(INQUIRY);
            bytes.push(evpd as u8);
            bytes.push(page_code);
            bytes.extend_from_slice(alloc_len.to_be_bytes().as_slice());
        }
        ScsiCommand::TestUnitReady => {
            bytes.push(TEST_UNIT_READY);
        }
        ScsiCommand::RequestSense { desc, alloc_len } => {
            bytes.push(REQUEST_SENSE);
            bytes.push(desc as u8);
            bytes.extend_from_slice([0; 2].as_slice());
            bytes.push(alloc_len);
        }
        ScsiCommand::ModeSense6 {
            dbd,
            page_control,
            page_code,
            subpage_code,
            alloc_len,
        } => {
            bytes.push(MODE_SENSE_6);
            bytes.push((dbd as u8) << 4);
            bytes.push(((page_control as u8) << 6) & (page_code & 0b00111111));
            bytes.push(subpage_code);
            bytes.push(alloc_len);
        }
        ScsiCommand::ModeSense10 {
            dbd,
            page_control,
            page_code,
            subpage_code,
            alloc_len,
        } => {
            bytes.push(MODE_SENSE_10);
            bytes.push((dbd as u8) << 4);
            bytes.push(((page_control as u8) << 6) & (page_code & 0b00111111));
            bytes.push(subpage_code);
            bytes.extend_from_slice([0; 3].as_slice());
            bytes.extend_from_slice(alloc_len.to_be_bytes().as_slice());
        }
        ScsiCommand::ReadCapacity10 => {
            bytes.push(READ_CAPACITY_10);
        }
        ScsiCommand::ReadCapacity16 { alloc_len } => {
            bytes.push(READ_CAPACITY_16);
            bytes.extend_from_slice([0; 10].as_slice());
            bytes.extend_from_slice(alloc_len.to_be_bytes().as_slice());
        }
        ScsiCommand::Read { lba, len } => {
            bytes.push(READ_10);
            bytes.push(0);
            bytes.extend_from_slice((lba as u32).to_be_bytes().as_slice());
            bytes.push(0);
            bytes.extend_from_slice((len as u16).to_be_bytes().as_slice());
        }
        ScsiCommand::Write { lba, len } => {
            bytes.push(WRITE_10);
            bytes.push(0);
            bytes.extend_from_slice((lba as u32).to_be_bytes().as_slice());
            bytes.push(0);
            bytes.extend_from_slice((len as u16).to_be_bytes().as_slice());
        }
        ScsiCommand::ReadFormatCapacities { alloc_len } => {
            bytes.push(READ_FORMAT_CAPACITIES);
            bytes.extend_from_slice([0; 6].as_slice());
            bytes.extend_from_slice(alloc_len.to_be_bytes().as_slice());
        }
    }
    bytes
}
