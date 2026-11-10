"""Find the standalone controller root."""
import os
from pathlib import Path
def find_repo_root(marker="examples/deploy_newurdf.yaml"):
    roots=[os.environ.get("DOG_MOTION_ROOT"),str(Path(__file__).resolve().parents[1]),os.getcwd()]
    for root in roots:
        if root is None:continue
        for path in [Path(root),*Path(root).parents]:
            if (path/marker).is_file():return str(path)
    raise RuntimeError("找不到控制器目录，请设置 DOG_MOTION_ROOT")
