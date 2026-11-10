import tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from usb_reconnect import USBReconnect,device_identity


class ReconnectTests(unittest.TestCase):
    def session(self):
        d=USBReconnect();d.setup_usb_reconnect('/tmp/test-only')
        d.ctrl=Mock();d.ctrl.is_enabled=False;d.ctrl.is_moving=False
        d.ctrl.legs=[{'code':k,'port':'/nonexistent/'+k} for k in ['FL','FR','RR','RL']]
        d.ctrl.buses=[SimpleNamespace(stop=Mock(),_thread=None,_port=object()) for _ in range(4)]
        d.ctrl.connect.return_value=12
        d.cfg={'imu':{'device':'/nonexistent/imu','parser':'parser.py'},'gamepad':{'device':'/nonexistent/pad'}}
        d.imu=Mock();d.imu.thread.is_alive.return_value=False;d.imu_error=None
        d.pad=Mock();d.pad.thread.is_alive.return_value=False
        d.close_log=Mock();d.latch_fault=Mock();d.snaps=Mock(return_value=[])
        d.post_fault_captures=[];d.fault=None;d.mode='IDLE';d.session_id='old'
        d.initialized=True;d.walk_unlocked=True;d.session_pd_verified=True;d.maps={'old':1}
        d.usb_signature=('old',)*5;d.serial_signature=Mock(return_value=('new',)*5)
        return d

    def reconnect(self,d):
        with patch('usb_reconnect.HI12') as imu,patch('usb_reconnect.QueuedJoystick') as pad,patch('builtins.print'):
            d.usb_reconnect_step(force=True)
        return imu,pad

    def test_identity_changes_even_if_tty_name_reused(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'ttyUSB0';p.touch();a=device_identity(p)
            p.rename(Path(t)/'old');p.touch();self.assertNotEqual(a,device_identity(p))
            p.unlink();self.assertIsNone(device_identity(p))

    def test_unchanged_devices_do_not_reconnect_on_motor_reply_loss(self):
        d=self.session();d.serial_signature.return_value=d.usb_signature
        self.assertFalse(d.usb_reconnect_step(force=True));d.ctrl.connect.assert_not_called()

    def test_changed_devices_reopen_disabled_and_invalidate_references(self):
        d=self.session();old=d.ctrl.buses.copy();imu,pad=self.reconnect(d)
        for bus in old:bus.stop.assert_called_once();self.assertIsNone(bus._port)
        d.ctrl.connect.assert_called_once();imu.return_value.start.assert_called_once();pad.assert_called_once()
        self.assertFalse(d.initialized);self.assertFalse(d.walk_unlocked);self.assertFalse(d.session_pd_verified)
        self.assertEqual(d.maps,{});self.assertNotEqual(d.session_id,'old');self.assertFalse(d.usb_pending)
        self.assertEqual(d.usb_generation,1);d.ctrl.enable.assert_not_called();d.ctrl.zero.assert_not_called()

    def test_fault_survives_reconnect(self):
        d=self.session();d.fault='old motor fault';d.mode='FAULT';self.reconnect(d)
        self.assertEqual(d.fault,'old motor fault');self.assertEqual(d.mode,'FAULT')

    def test_reconnect_retains_continuous_neutral_and_small_stick_response(self):
        d=self.session();self.reconnect(d)
        gate=d.stick_gate
        self.assertFalse(gate.stop_on_neutral)
        self.assertEqual(gate.start_threshold,0.)
        self.assertFalse(d.walk_unlocked)
        self.assertIsNone(gate.step([0]*4,[],True,False))
        self.assertEqual(gate.step([0,.2,0,0],[],True,False),'play')
        for axes in ([0]*4,[.3,0,0,0],[0]*4,[0,0,0,-.3]):
            self.assertIsNone(gate.step(axes,[],False,True))

    def test_enabled_disconnect_latches_before_any_reopen(self):
        d=self.session();d.ctrl.is_enabled=True;self.reconnect(d)
        d.latch_fault.assert_called_once();d.ctrl.connect.assert_not_called()
        self.assertEqual(d.mode,'FAULT')

    def test_motion_blocks_reopen_even_if_enabled_flag_false(self):
        d=self.session();d.ctrl.is_moving=True;self.reconnect(d)
        d.latch_fault.assert_called_once();d.ctrl.connect.assert_not_called()

    def test_single_enabled_axis_also_blocks_reopen(self):
        d=self.session();d.snaps.return_value=[{'enabled':True}]
        self.reconnect(d);d.latch_fault.assert_called_once();d.ctrl.connect.assert_not_called()

    def test_missing_devices_wait_without_opening_partial_bus(self):
        d=self.session();d.serial_signature.return_value=('new',None,'new','new','new')
        self.reconnect(d);d.ctrl.connect.assert_not_called();self.assertTrue(d.usb_pending)
        self.assertFalse(d.initialized);self.assertEqual(d.usb_status['state'],'waiting_devices')

    def test_stop_failure_never_opens_second_bus(self):
        d=self.session();d.ctrl.buses[0].stop.side_effect=RuntimeError('thread alive')
        self.reconnect(d);d.ctrl.connect.assert_not_called();self.assertTrue(d.usb_pending)
        self.assertEqual(d.usb_status['state'],'retrying')

    def test_missing_at_start_keeps_service_available(self):
        d=self.session();d.serial_signature.return_value=(None,)*5
        self.assertEqual(d.connect_hardware(),0);d.ctrl.connect.assert_not_called()
        self.assertTrue(d.usb_pending)

    def test_retry_is_rate_limited(self):
        d=self.session();d.usb_next_check=float('inf')
        self.assertFalse(d.usb_reconnect_step());d.serial_signature.assert_not_called()


if __name__=='__main__':unittest.main()
