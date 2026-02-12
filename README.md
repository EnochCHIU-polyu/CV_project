<div align="center">
  <img src="resources/mmpose-logo.png" width="450"/>
  <div>&nbsp;</div>
  <div align="center">
    <b>OpenMMLab website</b>
    <sup>
      <a href="https://openmmlab.com">
        <i>HOT</i>
      </a>
    </sup>
    &nbsp;&nbsp;&nbsp;&nbsp;
    <b>OpenMMLab platform</b>
    <sup>
      <a href="https://platform.openmmlab.com">
        <i>TRY IT OUT</i>
      </a>
    </sup>
  </div>
  <div>&nbsp;</div>

[![Documentation](https://readthedocs.org/projects/mmpose/badge/?version=latest)](https://mmpose.readthedocs.io/en/latest/?badge=latest)
[![actions](https://github.com/open-mmlab/mmpose/workflows/merge_stage_test/badge.svg)](https://github.com/open-mmlab/mmpose/actions)
[![codecov](https://codecov.io/gh/open-mmlab/mmpose/branch/latest/graph/badge.svg)](https://codecov.io/gh/open-mmlab/mmpose)
[![PyPI](https://img.shields.io/pypi/v/mmpose)](https://pypi.org/project/mmpose/)
[![LICENSE](https://img.shields.io/github/license/open-mmlab/mmpose.svg)](https://github.com/open-mmlab/mmpose/blob/main/LICENSE)
[![Average time to resolve an issue](https://isitmaintained.com/badge/resolution/open-mmlab/mmpose.svg)](https://github.com/open-mmlab/mmpose/issues)
[![Percentage of issues still open](https://isitmaintained.com/badge/open/open-mmlab/mmpose.svg)](https://github.com/open-mmlab/mmpose/issues)
[![Open in OpenXLab](https://cdn-static.openxlab.org.cn/app-center/openxlab_demo.svg)](https://openxlab.org.cn/apps?search=mmpose)

[📘Documentation](https://mmpose.readthedocs.io/en/latest/) |
[🛠️Installation](https://mmpose.readthedocs.io/en/latest/installation.html) |
[👀Model Zoo](https://mmpose.readthedocs.io/en/latest/model_zoo.html) |
[📜Papers](https://mmpose.readthedocs.io/en/latest/model_zoo_papers/algorithms.html) |
[🆕Update News](https://mmpose.readthedocs.io/en/latest/notes/changelog.html) |
[🤔Reporting Issues](https://github.com/open-mmlab/mmpose/issues/new/choose) |
[🔥RTMPose](/projects/rtmpose/)

</div>

## 🛠️ Environment Setup & VS Code Configuration

### 1. Installation (conda)

Follow the [official installation guide](https://mmpose.readthedocs.io/en/latest/installation.html) for detailed instructions.

**Step 0. Install Miniconda**
Download and install Miniconda from the [official website](https://docs.conda.io/en/latest/miniconda.html).

**Step 1. Create and activate a conda environment**

```bash
conda create --name openmmlab python=3.8 -y
conda activate openmmlab
```

**Step 2. Install PyTorch**
Follow the [official PyTorch instructions](https://pytorch.org/get-started/locally/) for your specific hardware.
_Example for CPU (macOS/Linux):_

```bash
conda install pytorch torchvision cpuonly -c pytorch
```

**Step 3. Install MMEngine, MMCV, and MMDetection**

```bash
pip install -U openmim
mim install mmengine
mim install "mmcv>=2.0.1"
mim install "mmdet>=3.1.0"
```

**Step 4. Install MMPose from source**

```bash
git clone https://github.com/open-mmlab/mmpose.git
cd mmpose
pip install -r requirements.txt
pip install -v -e .
```

### 2. VS Code Setup

To configure Visual Studio Code to use this environment:

1. Open this project folder in VS Code.
2. Install the **Python** extension (by Microsoft) if not already installed.
3. Open the **Command Palette** with `Cmd+Shift+P` (macOS) or `Ctrl+Shift+P` (Windows/Linux).
4. Type and select: `Python: Select Interpreter`.
5. Choose the **openmmlab** conda environment from the list.
   - It usually looks like `~/miniconda3/envs/openmmlab/bin/python`.
6. Any new terminal you open in VS Code will now automatically activate this environment.
