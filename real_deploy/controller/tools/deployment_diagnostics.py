"""Read-only CLI diagnostics. Never retries a motion command or clears a fault."""
import json
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


def watch_runtime(call,stream,expected_session=None):
    """Follow status only; never sends enable, stop, unlock or retry commands."""
    print('持续显示运行状态；发生故障会立即打印诊断。Ctrl+C 仅退出日志，不停止机器人；B 趴下，LB+RB+A 急停。',file=stream,flush=True)
    previous=None
    try:
        while True:
            response=call({'command':'status'})
            if not response.get('ok'):return response
            s=response['result'];d=s.get('deployment',{})
            if expected_session is not None and s.get('session')!=expected_session:
                return dict(ok=False,error='后台会话已改变，停止跟踪；没有自动重新解锁')
            expected_session=s.get('session')
            state=(d.get('mode'),s.get('enabled'),s.get('online'),d.get('walk_unlocked'))
            if state!=previous:
                stamp=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S')
                print(f'[{stamp}] 模式={state[0]}，使能={state[1]}，在线={state[2]}/12，行走解锁={state[3]}',file=stream,flush=True)
                previous=state
            if s.get('fault') or d.get('mode')=='FAULT':
                return dict(ok=False,error=s.get('fault') or d.get('last_policy_error') or '后台进入FAULT',
                            runtime_fault=True,fault_detail=s.get('fault_detail'))
            if not d.get('walk_unlocked') or (not s.get('enabled') and not s.get('moving')):
                return dict(ok=True,result={'message':'行走已取消解锁或电机已停用，结束日志跟踪'})
            time.sleep(.2)
    except KeyboardInterrupt:
        return dict(ok=True,result={'message':'已退出日志跟踪；未向机器人发送控制命令'})
    except (OSError,ValueError) as exc:
        return dict(ok=False,error=f'运行中与后台失去连接：{exc}',outcome_unknown=True)


def tail_log(path, lines=20):
    try:
        with path.open('rb') as f:
            f.seek(0,2);f.seek(max(0,f.tell()-16384))
            return f.read().decode('utf-8',errors='replace').splitlines()[-lines:]
    except OSError as exc:return [f'无法读取日志：{exc}']


def report_failure(command,result,call,root,stream):
    """Print/save the command error, live status and bounded historical log tail."""
    root=Path(root);now=datetime.now(ZoneInfo('Asia/Shanghai'))
    print(f'\n[{now.isoformat(timespec="seconds")}] {command} 执行失败',file=stream)
    print('原因：'+str(result.get('error') or result.get('result',{}).get('message') or result),file=stream)
    if result.get('outcome_unknown'):
        print('请求结果尚未确认；没有自动重发命令，请先查看 status。',file=stream)
    diagnostic=dict(time=now.isoformat(),command=command,response=result)
    unavailable=None
    try:
        response=call({'command':'status'})
        if not response.get('ok'):raise RuntimeError(response.get('error','status失败'))
        s=response['result'];d=s.get('deployment',{})
        diagnostic['status']=s
        print(f'当前：在线 {s.get("online")}/12，模式 {d.get("mode")}，使能 {s.get("enabled")}，'
              f'已读零 {s.get("initialized")}，本次静态检查通过 {d.get("session_pd_verified")}',file=stream)
        for label,value in [('锁存故障',s.get('fault')),('策略错误',d.get('last_policy_error')),
                            ('手柄错误',d.get('gamepad_error')),('静态检查结果',d.get('servo_check_result'))]:
            if value:print(f'{label}：{value}',file=stream)
        usb=d.get('usb_reconnect')
        if usb:print(f'USB重连：{usb.get("state")}，{usb.get("message")}',file=stream)
        for i,m in enumerate(s.get('motors',[])):
            if not m.get('online') or m.get('err') or m.get('consecutive_failures',0):
                leg=s.get('leg_order',['?']*4)[i//3]
                age=f'{m.get("feedback_age_s",0)*1000:.1f} ms' if m.get('valid_frames',0) else '本次启动尚无有效回传'
                print(f'异常轴：{leg} ID{m.get("id")}，在线={m.get("online")}，'
                      f'错误码={m.get("err")}（离线时为旧值），反馈={age}，'
                      f'连续失败={m.get("consecutive_failures",0)}，原因={m.get("last_failure_reason")}',file=stream)
        detail=s.get('fault_detail') or {}
        if detail.get('report_path'):print('故障现场记录：'+detail['report_path'],file=stream)
    except (OSError,ValueError,KeyError,RuntimeError) as exc:
        diagnostic['status_error']=str(exc)
        # ConnectionRefused/FileNotFound 只说明没有进程在监听套接字，和电机、
        # 清故障逻辑无关；先说这一句，免得把下面的历史日志当成故障原因。
        if isinstance(exc,(FileNotFoundError,ConnectionRefusedError)):
            unavailable='后台服务未运行（%s）：本次失败与电机、清故障逻辑无关。'%exc
            print(unavailable,file=stream)
            print('先启动后台：./dognew serve --gamepad\n'
                  '看到 {"online": 12, ...} 后再执行 clear-fault',file=stream)
        else:
            print(f'后台状态不可用：{exc}\n后台重启用：./dognew serve --gamepad',file=stream)
    logs=list((root/'reports').glob('newurdf_service*.log'))
    log=max(logs,key=lambda p:p.stat().st_mtime) if logs else root/'reports/newurdf_service.log'
    diagnostic['service_log_path']=str(log)
    diagnostic['service_log_tail']=tail_log(log)
    stamp=datetime.fromtimestamp(log.stat().st_mtime,ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds') if log.exists() else '无'
    hint='；后台未运行，以下内容不是本次失败原因' if unavailable else ''
    print(f'\n后台日志末尾（可能含历史错误{hint}；文件更新时间 {stamp}）：\n{log}',file=stream)
    print('\n'.join(diagnostic['service_log_tail']),file=stream)
    try:
        folder=root/'reports/command_errors';folder.mkdir(parents=True,exist_ok=True)
        path=folder/(now.strftime('%Y%m%d_%H%M%S_%f')+'_'+command+'.json')
        path.write_text(json.dumps(diagnostic,ensure_ascii=False,indent=2)+'\n')
        print(f'\n本次错误日志已保存：{path}',file=stream)
    except OSError as exc:print(f'错误日志保存失败：{exc}',file=stream)
