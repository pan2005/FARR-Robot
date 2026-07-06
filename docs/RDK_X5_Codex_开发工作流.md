# RDK X5 Codex 开发工作流

本文档约定后续 RDK X5 相关开发方式：**本地 PC 修改代码，使用 Makefile 部署到 RDK X5 测试**。尽量避免在交互式 SSH 会话中直接编辑板端文件，减少 PowerShell、SSH 管道、引号转义和长命令解析带来的问题。

## 基本原则

- 本地 PC 是主要开发环境。
- RDK X5 是部署与测试环境。
- 不在板子上通过交互式 SSH 直接改代码。
- 所有代码修改先发生在本地仓库。
- 使用 `make deploy` 同步代码到 RDK X5。
- 使用 `make rdk-test` 部署并执行板端测试。
- 使用 `make logs` 拉取板端近期日志。
- 长时间运行的命令应在板端 `tmux` 中启动。
- 不依赖长时间交互式 SSH 管道。

## 板端配置

SSH alias：

```bash
rdk
```

该 alias 应配置在本机：

```text
~/.ssh/config
```

远程项目路径：

```text
/home/sunrise/rdk_project
```

后续脚本、Makefile 和部署命令默认使用该路径作为 RDK X5 上的项目根目录。

## 常用命令

### 部署代码

```bash
make deploy
```

作用：

- 将本地项目代码同步到 RDK X5。
- 不直接在板端编辑源文件。

### 部署并测试

```bash
make rdk-test
```

作用：

- 先执行部署。
- 再运行板端测试脚本。
- Codex 后续调试时优先使用这个命令。

### 收集日志

```bash
make logs
```

作用：

- 拉取 RDK X5 最近日志。
- 用于分析运行失败、节点异常、模型推理错误或通信问题。

## 推荐调试循环

后续调试统一采用下面流程：

```text
1. 在本地修改代码
2. 运行 make rdk-test
3. 阅读终端输出和 make logs
4. 在本地修复问题
5. 再次运行 make rdk-test
6. 循环直到通过
```

不要采用：

```text
ssh rdk
vim remote_file.py
python3 remote_file.py
```

除非只是临时查看环境或执行非常短的只读检查。

## 长时间运行任务

长时间运行的任务应通过 `tmux` 在 RDK X5 上启动，例如：

```bash
ssh rdk
tmux new -s farr
```

在 `tmux` 内启动 ROS2、YOLO、Web 服务或日志采集任务。

断开后可重新连接：

```bash
ssh rdk
tmux attach -t farr
```

原则：

- 不把长时间运行服务挂在本地 SSH 管道上。
- 不依赖 PowerShell 长命令保持连接。
- 关键服务输出应写入日志文件，方便 `make logs` 收集。

## Codex 操作约定

Codex 后续执行 RDK 相关开发时，应遵守：

- 优先读取和修改本地文件。
- 修改完成后运行 `make rdk-test`。
- 不直接通过交互式 SSH 在板端改源代码。
- 不使用复杂 SSH 管道拼接长命令。
- 如果需要板端长期运行服务，写入脚本后通过 `tmux` 启动。
- 出现问题时先查看 `make rdk-test` 输出，再执行 `make logs`。

## Makefile 预期能力

项目根目录后续应提供 Makefile，至少包含：

```makefile
deploy:
	# sync local code to rdk:/home/sunrise/rdk_project

rdk-test: deploy
	# run board-side test script

logs:
	# collect recent logs from the board
```

具体同步方式可以使用：

```bash
rsync
scp
git archive
```

推荐优先使用 `rsync`，便于增量同步并排除 build/log/cache 文件。

## 目标

这个流程的目标是让后续 RDK X5 开发变成稳定、可复现的工程循环：

```text
本地改代码 -> 一键部署测试 -> 拉日志 -> 本地修复
```

避免再陷入：

- PowerShell 引号转义问题
- SSH 管道被截断
- 远程文件和本地文件不一致
- 板端临时修改忘记同步
- 长进程挂在 SSH 会话上导致调试状态混乱

