import io,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import Mock,MagicMock,patch
from deployment_diagnostics import report_failure,tail_log,watch_runtime
import deploy_newurdf


class DiagnosticsTests(unittest.TestCase):
    def test_arm_cli_follows_runtime_fault_by_default(self):
        connection=MagicMock();connection.__enter__.return_value=connection
        replies=[{'ok':True,'result':{'session':'one'}},self.state('PLAY'),self.state('FAULT','IMU超时')]
        connection.makefile.side_effect=[io.BytesIO(json.dumps(r).encode()+b'\n') for r in replies]
        with patch.object(sys,'argv',['dognew','arm-walk']),patch.object(deploy_newurdf.socket,'socket',return_value=connection),\
             patch('deployment_diagnostics.time.sleep'),patch('deployment_diagnostics.report_failure') as report,\
             patch('sys.stdout',new_callable=io.StringIO),patch('sys.stderr',new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as exc:deploy_newurdf.main()
        self.assertEqual(exc.exception.code,1)
        sent=[json.loads(c.args[0])['command'] for c in connection.sendall.call_args_list]
        self.assertEqual(sent,['arm-walk','status','status']);self.assertEqual(report.call_args.args[1]['error'],'IMU超时')

    def test_no_watch_is_explicit_opt_out(self):
        connection=MagicMock();connection.__enter__.return_value=connection
        connection.makefile.return_value=io.BytesIO(b'{"ok":true,"result":{"session":"one"}}\n')
        with patch.object(sys,'argv',['dognew','arm-walk','--no-watch']),\
             patch.object(deploy_newurdf.socket,'socket',return_value=connection),patch('sys.stdout',new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as exc:deploy_newurdf.main()
        self.assertEqual(exc.exception.code,0);self.assertEqual(connection.sendall.call_count,1)

    def state(self,mode='IDLE',fault=None,session='one',unlocked=True):
        return {'ok':True,'result':dict(session=session,online=12,enabled=not fault,moving=False,
            fault=fault,deployment={'mode':mode,'walk_unlocked':unlocked})}

    def test_fault_after_successful_unlock_is_reported(self):
        call=Mock(side_effect=[self.state(),self.state('PLAY'),self.state('FAULT','IMU反馈超过20毫秒')])
        out=io.StringIO()
        with patch('deployment_diagnostics.time.sleep'):
            result=watch_runtime(call,out,'one')
        self.assertFalse(result['ok']);self.assertIn('IMU',result['error']);self.assertTrue(result['runtime_fault'])
        self.assertIn('模式=PLAY',out.getvalue())
        self.assertTrue(all(c.args==({'command':'status'},) for c in call.call_args_list))

    def test_session_change_never_rearms(self):
        call=Mock(return_value=self.state(session='two'))
        result=watch_runtime(call,io.StringIO(),'one')
        self.assertFalse(result['ok']);self.assertIn('会话已改变',result['error']);self.assertEqual(call.call_count,1)

    def test_ctrl_c_only_exits_watcher(self):
        call=Mock(side_effect=KeyboardInterrupt())
        result=watch_runtime(call,io.StringIO(),'one')
        self.assertTrue(result['ok']);call.assert_called_once_with({'command':'status'})

    def test_unlock_cancelled_ends_watcher(self):
        result=watch_runtime(Mock(return_value=self.state(unlocked=False)),io.StringIO(),'one')
        self.assertTrue(result['ok'])

    def test_runtime_connection_failure_is_reported(self):
        result=watch_runtime(Mock(side_effect=ConnectionResetError('gone')),io.StringIO(),'one')
        self.assertFalse(result['ok']);self.assertIn('失去连接',result['error'])

    def test_failed_unlock_reports_axis_and_saves_without_control_request(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);out=io.StringIO()
            call=Mock(return_value={'ok':True,'result':dict(online=10,enabled=False,initialized=False,
                leg_order=['FL'],motors=[dict(id=2,online=False,err=0,valid_frames=0,consecutive_failures=12)],
                deployment={'mode':'IDLE','session_pd_verified':False})})
            report_failure('arm-walk',{'ok':False,'error':'先完成起立和检查'},call,root,out)
            self.assertIn('FL ID2',out.getvalue());self.assertIn('本次启动尚无有效回传',out.getvalue())
            call.assert_called_once_with({'command':'status'})
            saved=list((root/'reports/command_errors').glob('*.json'))
            self.assertEqual(len(saved),1);self.assertEqual(json.loads(saved[0].read_text())['command'],'arm-walk')

    def test_missing_service_keeps_diagnostics_available(self):
        with tempfile.TemporaryDirectory() as folder:
            out=io.StringIO();call=Mock(side_effect=FileNotFoundError('socket missing'))
            report_failure('arm-walk',{'ok':False,'error':'后台未启动'},call,folder,out)
            self.assertIn('./dognew serve --gamepad',out.getvalue())

    def test_ambiguous_timeout_warns_without_retry(self):
        with tempfile.TemporaryDirectory() as folder:
            out=io.StringIO();call=Mock(side_effect=TimeoutError('busy'))
            report_failure('arm-walk',{'ok':False,'error':'timeout','outcome_unknown':True},call,folder,out)
            self.assertIn('没有自动重发',out.getvalue())
            call.assert_called_once_with({'command':'status'})

    def test_log_tail_is_bounded_and_historical(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'reports').mkdir();p=root/'reports/newurdf_service.log'
            p.write_text('\n'.join(str(i) for i in range(100)))
            self.assertEqual(tail_log(p),[str(i) for i in range(80,100)])
            out=io.StringIO()
            report_failure('arm-walk',{'ok':False,'error':'bad'},Mock(side_effect=OSError()),root,out)
            self.assertIn('可能含历史错误',out.getvalue())

    def test_cli_missing_socket_has_no_traceback_and_nonzero_exit(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(deploy_newurdf,'ROOT',Path(folder)),\
             patch.object(sys,'argv',['dognew','arm-walk']),\
             patch.object(deploy_newurdf.socket,'socket',side_effect=FileNotFoundError('socket missing')),\
             patch('sys.stdout',new_callable=io.StringIO) as out,patch('sys.stderr',new_callable=io.StringIO) as err:
            with self.assertRaises(SystemExit) as exc:deploy_newurdf.main()
            self.assertEqual(exc.exception.code,1)
            self.assertFalse(json.loads(out.getvalue())['ok'])
            self.assertIn('arm-walk 执行失败',err.getvalue());self.assertNotIn('Traceback',err.getvalue())


if __name__=='__main__':unittest.main()
