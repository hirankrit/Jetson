# 📚 Documentation Index

> Complete documentation for Pepper Sorting Robot System

## 📖 Quick Navigation

### 🚀 Getting Started
- [../claude.md](../claude.md) - **Main quick reference** (start here!)
- [setup/jetson_setup.md](setup/jetson_setup.md) - Jetson Orin Nano setup guide

### 🔧 Setup & Installation
- [setup/jetson_setup.md](setup/jetson_setup.md) - JetPack 6.2.1, CUDA, PyTorch installation
- [setup/software_installation.md](setup/software_installation.md) - Software stack (coming soon)
- [setup/hardware_setup.md](setup/hardware_setup.md) - Camera & hardware setup (coming soon)

### 📊 Dataset
- [dataset/dataset_statistics.md](dataset/dataset_statistics.md) - Complete dataset statistics
- [dataset/collection_process.md](dataset/collection_process.md) - Collection workflow (coming soon)
- [dataset/annotation_workflow.md](dataset/annotation_workflow.md) - Annotation methods (coming soon)

### 🎓 Training
- [training/final_results.md](training/final_results.md) - **100-epoch training results** ⭐
- [training/training_process.md](training/training_process.md) - Training journey (coming soon)
- [training/gpu_troubleshooting.md](training/gpu_troubleshooting.md) - PyTorch/GPU fixes (coming soon)

### 🚀 Deployment
- [deployment/deployment_guide.md](deployment/deployment_guide.md) - **Production deployment guide** ⭐
- [deployment/tensorrt_optimization.md](deployment/tensorrt_optimization.md) - TensorRT guide (coming soon)
- [deployment/performance_tuning.md](deployment/performance_tuning.md) - Optimization tips (coming soon)

### 💻 Code
- [code/CODE_IMPROVEMENTS.md](code/CODE_IMPROVEMENTS.md) - **Best practices applied** ⭐
- [code/api_reference.md](code/api_reference.md) - Function documentation (coming soon)
- [code/scripts_guide.md](code/scripts_guide.md) - Script usage guide (coming soon)

### 📝 Development Log
- [development_log/daily_notes/2025-11-04.md](development_log/daily_notes/2025-11-04.md) - Latest session
- [development_log/week3_training.md](development_log/week3_training.md) - Week 3 summary (coming soon)

### 🔍 Troubleshooting
- [troubleshooting/common_issues.md](troubleshooting/common_issues.md) - FAQ (coming soon)
- [troubleshooting/gpu_issues.md](troubleshooting/gpu_issues.md) - GPU problems (coming soon)

## 📂 Documentation Structure

```
docs/
├── README.md (this file)           # Documentation index
│
├── setup/                          # Installation guides
│   ├── jetson_setup.md            ✅
│   ├── software_installation.md   🔜
│   └── hardware_setup.md          🔜
│
├── dataset/                        # Dataset documentation
│   ├── dataset_statistics.md      ✅
│   ├── collection_process.md      🔜
│   └── annotation_workflow.md     🔜
│
├── training/                       # Training documentation
│   ├── final_results.md           ✅
│   ├── training_process.md        🔜
│   └── gpu_troubleshooting.md     🔜
│
├── deployment/                     # Deployment guides
│   ├── deployment_guide.md        ✅
│   ├── tensorrt_optimization.md   🔜
│   └── performance_tuning.md      🔜
│
├── code/                           # Code documentation
│   ├── CODE_IMPROVEMENTS.md       ✅
│   ├── api_reference.md           🔜
│   └── scripts_guide.md           🔜
│
├── development_log/                # Development history
│   ├── daily_notes/
│   │   └── 2025-11-04.md          ✅
│   ├── week1_planning.md          🔜
│   ├── week2_dataset.md           🔜
│   └── week3_training.md          🔜
│
└── troubleshooting/                # Problem solving
    ├── common_issues.md            🔜
    ├── gpu_issues.md               🔜
    └── camera_issues.md            🔜
```

Legend: ✅ Complete | 🔜 Coming Soon

## 🎯 Documentation by Use Case

### "I want to set up a Jetson Orin Nano"
→ [setup/jetson_setup.md](setup/jetson_setup.md)

### "I want to train my own model"
→ [training/final_results.md](training/final_results.md)
→ [dataset/dataset_statistics.md](dataset/dataset_statistics.md)

### "I want to deploy the model"
→ [deployment/deployment_guide.md](deployment/deployment_guide.md)

### "I want to understand the code improvements"
→ [code/CODE_IMPROVEMENTS.md](code/CODE_IMPROVEMENTS.md)

### "I encountered an error"
→ [troubleshooting/common_issues.md](troubleshooting/common_issues.md) (coming soon)
→ [setup/jetson_setup.md](setup/jetson_setup.md) (for setup issues)

## 📊 Quick Stats

**Project Status:**
- Week 3/6 Complete ✅
- Model: 89.59% mAP@50
- Dataset: 805 images, 10 classes
- Training: 48 minutes (GPU)

**Documentation Status:**
- Total files: 25+ planned
- Complete: 6 ✅
- In progress: 19 🔜

## 🔗 External Resources

### Official Documentation
- [NVIDIA Jetson Documentation](https://developer.nvidia.com/embedded/jetson-orin-nano-devkit-user-guide)
- [JetPack Documentation](https://developer.nvidia.com/embedded/jetpack)
- [Ultralytics YOLO Documentation](https://docs.ultralytics.com/)
- [PyTorch Documentation](https://pytorch.org/docs/stable/index.html)

### GitHub Repository
- [Project Repository](https://github.com/hirankrit/Jetson)

### Historical Archive
- [Full Development History](../archive/claude_md_archive/claude_full_2025-11-04.md) (4,546 lines)

## 📝 Contributing

Documentation updates welcome! When adding new documentation:

1. Create file in appropriate subfolder
2. Update this README.md index
3. Cross-reference related docs
4. Add to commit message

## ⚡ Quick Commands

```bash
# View all documentation files
find docs/ -name "*.md" | sort

# Search documentation
grep -r "keyword" docs/

# Count documentation lines
find docs/ -name "*.md" -exec wc -l {} +
```

---

**Last Updated:** November 4, 2025
**Status:** Active Development
**Contact:** See main README.md
