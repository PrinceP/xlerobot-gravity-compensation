#!/usr/bin/env python3
"""Copy the project page (docs/) into the princep.github.io blog post folder.

    python tools/publish_blog.py ~/path/to/princep.github.io [--updated 2026-11-25]

Writes blogs/2026-11-24-XLeRobot-Gravity-Compensation/{index.html,data/,media/} with the
site's Prince / Blogs / Projects header instead of the standalone one. The blog list and
projects cards already link to that folder. Commit and push in the site repo yourself.
"""
import argparse
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
POST = "blogs/2026-11-24-XLeRobot-Gravity-Compensation"
PUBLISHED = "2026-11-24"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("site", type=Path, help="checkout of PrinceP/princep.github.io")
    ap.add_argument("--updated", help="date shown as 'updated' after the publish date")
    a = ap.parse_args()
    dst = a.site / POST
    if not (a.site / "blogs/blog.html").exists():
        raise SystemExit(f"{a.site} doesn't look like the princep.github.io checkout")
    dst.mkdir(parents=True, exist_ok=True)
    for d in ("data", "media"):
        if (dst / d).exists():
            shutil.rmtree(dst / d)
        shutil.copytree(ROOT / "docs" / d, dst / d)
    s = (ROOT / "docs/index.html").read_text()
    s = s.replace('<div class="brand">XLeRobot gravity compensation</div>',
                  '<div class="brand"><a href="../../index.html" style="color:inherit;text-decoration:none">Prince</a> '
                  '<span class="muted">/</span> <a href="../blog.html" style="color:inherit;text-decoration:none">Blogs</a> '
                  '<span class="muted">/</span> <a href="../../projects/projects.html" style="color:inherit;text-decoration:none">Projects</a></div>')
    s = s.replace("<title>XLeRobot Gravity Compensation</title>", "<title>XLeRobot Gravity Compensation - Prince</title>")
    date = PUBLISHED + (f" · updated {a.updated}" if a.updated else "")
    s = s.replace("  <h1>Two robot arms holding a board level, soft to the touch</h1>",
                  f'  <p class="small muted" style="margin:32px 0 -24px">{date}</p>\n'
                  "  <h1>Two robot arms holding a board level, soft to the touch</h1>", 1)
    assert "Prince</a>" in s and date in s, "header transform did not apply"
    (dst / "index.html").write_text(s)
    print("wrote", dst)


if __name__ == "__main__":
    main()
