#!/usr/bin/env bash
#
# 把旧的 scripts/ 结构迁移到新的 src/ tools/ archive/ 结构。
#
# 用法（在任意一台能 clone 仓库的电脑上）：
#   1. git clone <你的仓库地址>
#   2. cd <仓库>
#   3. git checkout -b refactor
#   4. 把我打包的 pointcloud-to-mesh.zip 里的
#      src/ tools/ archive/*/README.md docs/ README.md requirements.txt
#      解压覆盖进仓库根目录
#   5. bash migrate_to_new_structure.sh
#   6. 检查无误后： git commit -m "refactor: 整理为 FAST-LIVO 后处理工具结构"
#                  git push -u origin refactor
#
# 脚本只用 git mv / git rm，所有改动都进暂存区，commit 前可随时 reset 反悔。

set -e

echo "==> 检查是否在 git 仓库根目录"
if [ ! -d .git ]; then
  echo "错误：当前目录不是 git 仓库根目录。请先 cd 进仓库。"
  exit 1
fi

echo "==> 建立归档目录"
mkdir -p archive/icp_selffuse archive/slam_pose_fusion tools

# ----------------------------------------------------------------------
# 归档：早期 ICP 自融合
# ----------------------------------------------------------------------
echo "==> 归档 ICP 自融合脚本"
[ -f scripts/bag_icp_fusion.py ] && git mv scripts/bag_icp_fusion.py archive/icp_selffuse/ || echo "  跳过 bag_icp_fusion.py（不存在）"
[ -f scripts/extract_colored_ply_frames.py ] && git mv scripts/extract_colored_ply_frames.py archive/icp_selffuse/ || echo "  跳过 extract_colored_ply_frames.py"

# ----------------------------------------------------------------------
# 归档：KISS-ICP / 外部 pose 融合
# ----------------------------------------------------------------------
echo "==> 归档 SLAM pose 融合脚本"
[ -f scripts/merge_with_pose.py ] && git mv scripts/merge_with_pose.py archive/slam_pose_fusion/ || echo "  跳过 merge_with_pose.py"
[ -f scripts/scan.py ] && git mv scripts/scan.py archive/slam_pose_fusion/ || echo "  跳过 scan.py"
[ -f scripts/tsdf_fusion.py ] && git mv scripts/tsdf_fusion.py archive/slam_pose_fusion/ || echo "  跳过 tsdf_fusion.py"

# ----------------------------------------------------------------------
# tools：合并/迁移检查脚本
# 注：新的 tools/inspect_bag.py 已经合并了 info/check/inspect_* 的功能，
#     所以旧的这几个直接删掉（功能没丢，在 inspect_bag.py 里）。
#     view_pointcloud.py 的功能也并入了 tools/view.py。
# ----------------------------------------------------------------------
echo "==> 删除已被 tools/ 合并取代的旧检查脚本"
for f in info.py check.py inspect_bag_topics.py inspect_pointcloud_fields.py view_pointcloud.py; do
  [ -f "scripts/$f" ] && git rm "scripts/$f" || echo "  跳过 $f"
done

# ----------------------------------------------------------------------
# 删除：危险且已无用的脚本
# ----------------------------------------------------------------------
echo "==> 删除危险/无用脚本"
[ -f scripts/convert_format.py ] && git rm scripts/convert_format.py || echo "  跳过 convert_format.py"

# 旧的 make_mesh.py 被 src/make_colored_mesh.py 取代，归档保留作参考
[ -f scripts/make_mesh.py ] && git mv scripts/make_mesh.py archive/ || echo "  跳过 make_mesh.py"

# ----------------------------------------------------------------------
# 清理空的 scripts/ 目录
# ----------------------------------------------------------------------
echo "==> 清理 scripts/ 残留"
if [ -d scripts ]; then
  remaining=$(find scripts -type f | wc -l)
  if [ "$remaining" -eq 0 ]; then
    rmdir scripts 2>/dev/null && echo "  scripts/ 已空，删除" || echo "  scripts/ 非空，保留"
  else
    echo "  scripts/ 还剩 $remaining 个文件，请手动检查："
    find scripts -type f
  fi
fi

echo ""
echo "==> 迁移完成。现在的暂存区状态："
git status --short
echo ""
echo "确认无误后："
echo "  git commit -m \"refactor: 整理为 FAST-LIVO 后处理工具结构\""
echo "  git push -u origin refactor"
echo ""
echo "想反悔（commit 前）： git reset --hard HEAD  （会丢未提交改动，慎用）"
