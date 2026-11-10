"""Real HI12 parser with an in-memory serial source; never opens a device."""
import struct,unittest
from pathlib import Path
from unittest.mock import patch
from dog_motion.deploy_inputs import HI12

ROOT=Path(__file__).resolve().parents[1]


class SerialChunks:
    def __init__(self,imu,chunks):self.imu=imu;self.chunks=list(chunks);self.requests=[]
    @property
    def in_waiting(self):return len(self.chunks[0]) if self.chunks else 0
    def read(self,n):
        self.requests.append(n);chunk=self.chunks.pop(0)
        self.assert_size=n>=max(1,len(chunk))
        if not self.chunks:self.imu.running=False
        return chunk


class HI12StreamingTests(unittest.TestCase):
    def setUp(self):self.imu=HI12('/unused',ROOT/'vendor/hi12/hi12_monitor.py')
    def frame(self,ts):
        p=self.imu.parser
        b=bytearray(p.FRAME_LEN);b[:2]=p.SYNC;b[2:4]=p.PAYLOAD_LEN.to_bytes(2,'little')
        b[p.HDR_SIZE+p.P_TAG]=p.TAG_IMU_FLOAT
        struct.pack_into('<I',b,p.HDR_SIZE+p.P_TS,ts)
        struct.pack_into('<4f',b,p.HDR_SIZE+p.P_QUAT,1,0,0,0)
        b[4:6]=p.frame_crc(bytes(b)).to_bytes(2,'little')
        return bytes(b)
    def run_chunks(self,chunks):
        self.imu.serial=SerialChunks(self.imu,chunks);self.imu.running=True
        with patch('dog_motion.deploy_inputs.time.monotonic',return_value=100.):self.imu.loop()
    def test_complete_frame_is_parsed_without_waiting_for512_bytes(self):
        b=self.frame(1000);self.run_chunks([b])
        self.assertEqual(self.imu.serial.requests,[len(b)])
        self.assertEqual(self.imu.latest['ts_ms'],1000)
        self.assertEqual(self.imu.latest['monotonic_s'],100.)
    def test_idle_and_partial_frames_use_incremental_parser(self):
        b=self.frame(1000);self.run_chunks([b'',b[:4],b[4:]])
        self.assertEqual(self.imu.serial.requests,[1,4,len(b)-4]);self.assertEqual(self.imu.latest['valid_frames'],1)
    def test_duplicate_timestamp_does_not_refresh_age(self):
        self.run_chunks([self.frame(1000),self.frame(1000)])
        self.assertEqual(self.imu.latest['valid_frames'],1)
    def test_backwards_timestamp_remains_error(self):
        self.run_chunks([self.frame(1000),self.frame(999)])
        self.assertIn('时间戳回退',self.imu.error)
    def test_crc_failure_is_rejected(self):
        bad=bytearray(self.frame(1000));bad[-1]^=1
        self.run_chunks([bytes(bad),self.frame(1010)])
        self.assertEqual(self.imu.latest['valid_frames'],1);self.assertEqual(self.imu.latest['bad_crc'],1)


if __name__=='__main__':unittest.main()
