"""Apply the operator-authorized CPU affinity to Clash's WebKit UI only."""
import json
import os
from pathlib import Path
import sys


def clash_ui_process(pid, proc=Path('/proc')):
    try:
        child=proc/str(pid)
        if child.stat().st_uid != os.getuid():
            return False
        if child.joinpath('comm').read_text().strip() != 'WebKitWebProces':
            return False
        status=child.joinpath('status').read_text().splitlines()
        parent=int(next(line.split()[1] for line in status if line.startswith('PPid:')))
        return (proc/str(parent)/'comm').read_text().strip() == 'clash-verge'
    except (OSError, ValueError, StopIteration):
        return False


def limit_clash_ui():
    cpus=sorted(os.sched_getaffinity(0))
    if len(cpus)<=2:
        return
    chosen=set(cpus[:2])
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or not clash_ui_process(int(entry.name)):
            continue
        pid=int(entry.name)
        try:
            # New threads inherit the main thread's affinity.
            os.sched_setaffinity(pid,chosen)
            for task in (entry/'task').iterdir():
                try:os.sched_setaffinity(int(task.name),chosen)
                except ProcessLookupError:pass
            print(json.dumps(dict(event='clash_ui_cpu_limited',pid=pid,
                                  cpus=sorted(chosen)),ensure_ascii=False),
                  file=sys.stderr,flush=True)
        except OSError as exc:
            print(f'Clash界面CPU限制未应用：{exc}',file=sys.stderr,flush=True)


if __name__=='__main__':
    limit_clash_ui()
