# 问题三离线测试代码

本文件夹用于测试，不是另一套求解算法。这里集中存放本地模拟器 Mock、几何和调度测试、随机场景生成、批量回归及测试报告整理脚本。

测试直接调用上一级目录中的生产算法，使用确定性的本地场景，不连接官方模拟器，也不占用队伍测试会话。正常执行 `problem3.exe --robot-id ...` 时不会运行这些测试。

## 文件用途

| 文件 | 测试用途 |
| --- | --- |
| `self_test.hpp` | 离线测试入口、场景生成、完整任务验证、配对比较和统计报告 |
| `mock_transport.hpp` | 本地 Mock，模拟测向、接收、清除、频道及虚拟计时 |
| `test_support.hpp` | 断言工具和仅供测试使用的精确点路线参考求解器 |
| `geometry_tests.hpp` | 汇总各组测试，检查覆盖、定位、接收、计时及末尾五项指标 |
| `neighborhood_route_tests.hpp` | 区域路线规划与独立穷举结果的对比 |
| `unified_tests.hpp` | 搜索、补测、清除统一调度及状态转换 |
| `one_read_tests.hpp` | 一次补测区域、圆心候选及真实 Mock 测量 |
| `clear_region_tests.hpp` | 完整清除可行域、投影和认证清除 |
| `probe_clear_tests.hpp` | 受限试清除成功、失败、次数限制及后续补测 |
| `bearing_estimate_tests.hpp` | 全部测向的约束估计、角度环绕及病态输入 |
| `merge_reports.ps1` | 合并同版本且种子不重叠的离线配对测试报告 |

## 编译和运行

在 `problem3` 目录执行：

```powershell
.\build.ps1
.\problem3.exe --self-test --cases 100 --report tests\validation_100.json
```

头文件由 `main.cpp` 中的 `#include "tests/self_test.hpp"` 引入，不需要单独编译，也不需要新的测试可执行程序。`--self-test` 和 `--compare-with` 的用法保持不变。

使用保留的五例报告验证结果没有变化：

```powershell
.\problem3.exe --compare-with completion_output_check_5.json --cases 5 --report tests\regression_5.json
```

每次先运行几何和集成检查，再执行指定数量的完整场景。成功时输出 `self-test OK`，末尾显示清除数、源总数、总虚拟时间、清除比例和平均每源时间。批量汇总采用全部案例合计，不是某一个场景的成绩。

`--seed-start` 用于选择起始种子，配对基线必须包含所选种子。`--report` 会覆盖目标文件，不能指定为基线文件。测试结果是本地 Mock 数据，不是官方模拟器成绩。

## 合并报告

`merge_reports.ps1` 的输入和输出路径相对于 `problem3` 目录解析。历史默认分片已经在旧 JSON 清理中删除，因此需要显式指定实际存在的分片；脚本不会恢复历史报告。

```powershell
.\tests\merge_reports.ps1 -InputReports @('tests\part_0.json', 'tests\part_1.json') -OutputReport tests\merged.json
```

输入必须为同版本、同场景生成器的成功配对测试报告，而且种子不重复。合并报告是已有结果的整理，不会运行测试或连接模拟器。
