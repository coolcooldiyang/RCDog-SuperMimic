#!/usr/bin/env python3
"""只读电机反馈上位机：仅请求已有部署服务status，不打开任何硬件串口。"""
import argparse,csv,hashlib,json,os,signal,socket,sys,time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
from PyQt5.QtCore import Qt,QThread,QTimer,pyqtSignal
from PyQt5.QtGui import QColor,QFont,QPainter,QPen
from PyQt5.QtWidgets import (QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QLabel,
    QPushButton,QTableWidget,QTableWidgetItem,QHeaderView,QComboBox,QMessageBox)
from collections import deque
from feedback_stats import FeedbackStats,LEG_NAMES

ROOT=Path(__file__).resolve().parents[1]
CSV_FIELDS=['time','session','mode','enabled','fault','ipc_ms','leg','id','joint','online','err','temperature','age_ms','poll_gap_ms','io_ms','window_s','valid_delta','failed_delta','failure_percent','success_hz','failed_total','consecutive_failures','age_peak_ms','age_p95_ms','failure_reason','io_exception']

def socket_path():
    return '/tmp/rcdog27-%d-%s.sock'%(os.getuid(),hashlib.sha256(str(ROOT).encode()).hexdigest()[:10])

class Reader(QThread):
    received=pyqtSignal(dict,float,float)
    failed=pyqtSignal(str)
    def run(self):
        while not self.isInterruptionRequested():
            started=time.monotonic()
            try:
                with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
                    s.settimeout(.5);s.connect(socket_path())
                    # The only permitted IPC request in this monitor.
                    s.sendall(b'{"command":"status"}\n')
                    with s.makefile('rb') as f:r=json.loads(f.readline(262144))
                if not r.get('ok'):raise RuntimeError(r.get('error','状态请求失败'))
                now=time.monotonic();self.received.emit(r['result'],now,(now-started)*1000)
            except (OSError,ValueError,RuntimeError) as exc:self.failed.emit(str(exc))
            remaining=max(0.,.2-(time.monotonic()-started))
            self.msleep(round(remaining*1000))

class Trend(QWidget):
    def __init__(self):
        super().__init__();self.points=deque(maxlen=300);self.setMinimumHeight(145)
    def paintEvent(self,event):
        p=QPainter(self);p.fillRect(self.rect(),QColor('#f8fafc'));p.setRenderHint(QPainter.Antialiasing)
        p.setFont(QFont('Noto Sans CJK SC',8));line=p.fontMetrics().height()
        p.setPen(QColor('#334155'));p.drawText(12,line+6,'选中电机 · 最近60秒反馈年龄（ms）  红线：策略12 ms阈值')
        if not self.points:return
        points=list(self.points);end=points[-1][0];top=line+18;bottom=self.height()-line-8;left=65;right=self.width()-35
        values=[v for _,v in points if v is not None];maximum=max(20.,min(200.,max(values,default=20)*1.1))
        def y(v):return bottom-min(maximum,max(0.,v))/maximum*(bottom-top)
        p.setPen(QPen(QColor('#cbd5e1'),1));p.drawLine(left,bottom,right,bottom)
        for value in [0,12,maximum]:
            p.setPen(QColor('#64748b'));p.drawText(5,int(y(value))+4,f'{value:.0f}')
        p.setPen(QPen(QColor('#ef4444'),1,Qt.DashLine));p.drawLine(left,int(y(12)),right,int(y(12)))
        p.setPen(QPen(QColor('#2563eb'),2));last=None
        for t,v in points:
            if v is None:last=None;continue
            current=(int(right+(t-end)/60*(right-left)),int(y(v)))
            if last:p.drawLine(*last,*current)
            last=current
        p.setPen(QColor('#64748b'));p.drawText(left,bottom+line,'-60s');p.drawText(right-45,bottom+line,'现在')

class Monitor(QMainWindow):
    def __init__(self):
        super().__init__();self.setWindowTitle('RCDOG27 · 电机通信监视（只读）');self.resize(1460,850)
        self.stats=FeedbackStats();self.history={};self.latest=None;self.rows=[];self.last_received=None;self.csv_file=None;self.csv_writer=None;self.csv_path=None;self.sample_count=0
        base=QWidget();self.setCentralWidget(base);layout=QVBoxLayout(base);layout.setContentsMargins(20,16,20,16);layout.setSpacing(12)
        title=QLabel('电机通信监视');title.setFont(QFont('',22,QFont.Bold));layout.addWidget(title)
        note=QLabel('只读已有服务数据 · 不使能、不读零、不清故障 · 状态采样5 Hz');note.setStyleSheet('color:#64748b');layout.addWidget(note)
        self.summary=QLabel('正在连接部署服务……');self.summary.setWordWrap(True);self.summary.setStyleSheet('padding:12px;background:#e2e8f0;border-radius:6px;font-size:10pt');layout.addWidget(self.summary)
        controls=QHBoxLayout();self.window=QComboBox();self.window.addItems(['最近10秒','最近30秒','最近60秒']);self.window.currentIndexChanged.connect(self.change_window)
        controls.addWidget(QLabel('失败率统计窗口'));controls.addWidget(self.window)
        reset=QPushButton('重新统计（仅本窗口）');reset.clicked.connect(self.reset_stats);controls.addWidget(reset)
        self.record=QPushButton('开始记录 CSV');self.record.clicked.connect(self.toggle_record);controls.addWidget(self.record);controls.addStretch()
        self.connection=QLabel('等待数据');controls.addWidget(self.connection);layout.addLayout(controls)
        columns=['腿 / 端口','电机','状态 / 错误码','反馈年龄 ms','窗口峰值 ms','轮询间隔 ms','收发耗时 ms','窗口失败率 %','窗口失败 / 尝试','成功回传 Hz','累计失败','连续失败']
        self.table=QTableWidget(12,len(columns));self.table.setHorizontalHeaderLabels(columns);self.table.verticalHeader().hide();self.table.setEditTriggers(QTableWidget.NoEditTriggers);self.table.setSelectionBehavior(QTableWidget.SelectRows);self.table.setSelectionMode(QTableWidget.SingleSelection);self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents);self.table.horizontalHeader().setStretchLastSection(True)
        for i in range(12):
            self.table.setRowHeight(i,32)
            for j in range(len(columns)):self.table.setItem(i,j,QTableWidgetItem('—'))
        self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff);self.table.itemSelectionChanged.connect(self.select_axis);layout.addWidget(self.table)
        self.trend=Trend();self.trend.setFixedHeight(210);layout.addWidget(self.trend)
        self.detail=QLabel('选择一行查看该电机曲线和最近失败原因');self.detail.setWordWrap(True);layout.addWidget(self.detail)
        self.foot=QLabel('失败率 = 窗口内失败增量 ÷（成功增量 + 失败增量）。SDK失败不等同于已确认的线缆丢包。\n反馈年龄 = 距主机最后更新有效回传的时间；收发耗时包含USB、驱动和线程调度。峰值/曲线来自5 Hz抽样，可能漏掉短暂尖峰。');self.foot.setWordWrap(True);self.foot.setStyleSheet('color:#64748b;font-size:8pt');layout.addWidget(self.foot)
        self.setStyleSheet('QMainWindow{background:#ffffff} QTableWidget{border:1px solid #cbd5e1;gridline-color:#e2e8f0;alternate-background-color:#f8fafc} QHeaderView::section{background:#eef2f7;padding:8px;border:0;font-weight:bold} QPushButton{padding:7px 12px} QComboBox{padding:6px}')
        self.reader=Reader();self.reader.received.connect(self.update_status);self.reader.failed.connect(self.connection_error);self.reader.start()
        self.timer=QTimer(self);self.timer.timeout.connect(self.check_stale);self.timer.start(250)
        QTimer.singleShot(0,self.fit_table)

    def fit_table(self):
        # Qt clamps row heights according to font/DPI: use actual geometry.
        self.table.resizeColumnsToContents()
        content=sum(self.table.rowHeight(i) for i in range(12))
        bar=self.table.horizontalScrollBar().sizeHint().height() if self.table.horizontalHeader().length()>self.table.viewport().width() else 0
        self.table.setFixedHeight(content+self.table.horizontalHeader().height()+2*self.table.frameWidth()+bar+4)
    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'table'):QTimer.singleShot(0,self.fit_table)

    def change_window(self,index):
        self.stats.window_s=(10.,30.,60.)[index];self.reset_stats()
    def reset_stats(self):
        self.stats.reset();self.history.clear();self.trend.points.clear();self.trend.update()
    def toggle_record(self):
        if self.csv_file:
            self.csv_file.close();self.csv_file=None;self.csv_writer=None;self.record.setText('开始记录 CSV');self.record.setToolTip('已保存：'+str(self.csv_path));self.detail.setText('记录已保存：'+str(self.csv_path));return
        try:
            directory=ROOT/'reports/feedback_monitor';directory.mkdir(parents=True,exist_ok=True)
            self.csv_path=directory/(datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')+'.csv')
            self.csv_file=self.csv_path.open('x',encoding='utf-8-sig',newline='');self.csv_writer=csv.DictWriter(self.csv_file,fieldnames=CSV_FIELDS);self.csv_writer.writeheader()
            self.record.setText('停止记录 CSV');self.record.setToolTip(str(self.csv_path))
        except OSError as exc:QMessageBox.warning(self,'无法保存记录',str(exc))
    def update_status(self,status,now,ipc_ms):
        self.last_received=now;self.latest=status;self.sample_count+=1
        previous=self.stats.resets;self.rows=self.stats.update(status,now)
        if self.stats.resets!=previous:self.history.clear()
        d=status.get('deployment',{});fault=status.get('fault') or '无'
        imu=d.get('imu') or {};imuage=imu.get('age_s');imustr=f'{imuage*1000:.1f} ms' if imuage is not None else str(imu.get('error','无数据'))
        self.summary.setText(f"电机在线 {status.get('online',0)}/12   ·   出力 {'已使能' if status.get('enabled') else '未使能'}   ·   模式 {d.get('mode',status.get('mode','?'))}   ·   IMU {imustr}\n锁存故障：{fault}")
        self.connection.setText(f'已连接 · IPC {ipc_ms:.1f} ms');self.connection.setStyleSheet('color:#15803d')
        stamp=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='milliseconds')
        def fmt(v,precision=2):return '—' if v is None else f'{v:.{precision}f}'
        for i,row in enumerate(self.rows):
            key=(row['leg'],row['id']);hist=self.history.setdefault(key,deque(maxlen=300));hist.append((now,row['age_ms']))
            texts=[LEG_NAMES.get(row['leg'],row['leg']),f"ID{row['id']} {row['joint']}",('在线 / ' if row['online'] else '离线 / 旧码 ')+str(row['err']),fmt(row['age_ms']),fmt(row['age_peak_ms']),fmt(row['poll_gap_ms']),fmt(row['io_ms']),fmt(row['failure_percent'],3),f"{row['failed_delta']} / {row['valid_delta']+row['failed_delta']}",fmt(row['success_hz'],1),str(row['failed_total']),str(row['consecutive_failures'])]
            for j,value in enumerate(texts):
                item=self.table.item(i,j);item.setText(value);item.setTextAlignment(Qt.AlignCenter);item.setForeground(QColor('#0f172a'));item.setBackground(QColor('#ffffff' if i//3%2==0 else '#f8fafc'))
            bad=not row['online'] or bool(row['err']);late=row['age_ms'] is not None and row['age_ms']>12
            for j in ([2] if bad else [])+([3] if late else [])+([7,8] if row['failed_delta'] else []):
                self.table.item(i,j).setForeground(QColor('#b91c1c'));self.table.item(i,j).setBackground(QColor('#fee2e2'))
            if self.csv_writer:
                try:self.csv_writer.writerow(dict(row,time=stamp,session=status.get('session'),mode=d.get('mode'),enabled=status.get('enabled'),fault=status.get('fault'),ipc_ms=ipc_ms))
                except OSError as exc:
                    self.csv_file.close();self.csv_file=None;self.csv_writer=None;self.record.setText('开始记录 CSV');self.detail.setText('记录失败：'+str(exc))
        # A short or empty response must not leave old rows looking live.
        for i in range(len(self.rows),12):
            for j in range(self.table.columnCount()):self.table.item(i,j).setText('—')
        if self.csv_file:
            try:self.csv_file.flush()
            except OSError as exc:self.detail.setText('写入失败：'+str(exc))
        if self.table.currentRow()<0:self.table.selectRow(0)
        self.select_axis()
    def select_axis(self):
        i=self.table.currentRow()
        if not 0<=i<len(self.rows):return
        r=self.rows[i];self.trend.points=self.history.get((r['leg'],r['id']),deque(maxlen=300));self.trend.update()
        self.detail.setText(f"{LEG_NAMES[r['leg']]} · ID{r['id']} {r['joint']}   温度 {r['temperature']} °C   统计跨度 {r['window_s']:.1f}s   最近历史失败原因：{r['failure_reason'] or '无'} {r['io_exception'] or ''}"+(f'   CSV：{self.csv_path}' if self.csv_file else ''))
    def connection_error(self,error):
        self.connection.setText('服务断连 · 自动重试');self.connection.setStyleSheet('color:#b91c1c');self.summary.setText('无法读取部署服务：'+error+'\n未发送任何控制指令。请保留原控制服务运行。')
        self.mark_stale()
    def mark_stale(self):
        for i in range(12):
            for j in range(self.table.columnCount()):self.table.item(i,j).setForeground(QColor('#94a3b8'))
            self.table.item(i,2).setText('旧数据')
    def check_stale(self):
        if self.last_received is not None and time.monotonic()-self.last_received>1:
            self.mark_stale();self.connection.setText(f'旧数据 · {time.monotonic()-self.last_received:.1f}s未更新');self.connection.setStyleSheet('color:#b91c1c')
    def closeEvent(self,event):
        self.reader.requestInterruption();self.reader.wait(1500)
        if self.csv_file:self.csv_file.close()
        event.accept()

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--smoke-test',action='store_true');parser.add_argument('--screenshot',type=Path);args=parser.parse_args()
    app=QApplication(sys.argv[:1]);app.setFont(QFont('Noto Sans CJK SC',10));window=Monitor();window.showMaximized()
    signal.signal(signal.SIGTERM,lambda *_:window.close())
    if args.smoke_test:
        def finish():
            if args.screenshot:args.screenshot.parent.mkdir(parents=True,exist_ok=True);window.grab().save(str(args.screenshot))
            print(json.dumps(dict(samples=window.sample_count,rows=len(window.rows),table_viewport_height=window.table.viewport().height(),table_rows_height=sum(window.table.rowHeight(i) for i in range(12)),session=window.latest.get('session') if window.latest else None),ensure_ascii=False),flush=True)
            ok=window.sample_count>=3 and len(window.rows)==12 and window.table.viewport().height()>=sum(window.table.rowHeight(i) for i in range(12))
            window.close();app.exit(0 if ok else 1)
        QTimer.singleShot(3000,finish)
    return app.exec_()
if __name__=='__main__':raise SystemExit(main())
