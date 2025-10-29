#!/bin/bash
# 🗂️ Setup New Dataset Structure
# Backup old dataset and create fresh folders

echo "================================================================================"
echo "🗂️  DATASET SETUP - Backup Old & Create New"
echo "================================================================================"
echo ""

# Get timestamp for backup
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OLD_DIR="pepper_dataset_OLD_${TIMESTAMP}"

# Step 1: Backup existing dataset
if [ -d "pepper_dataset" ]; then
    echo "📦 Step 1: Backing up existing dataset..."
    echo "   Old: pepper_dataset/"
    echo "   New: ${OLD_DIR}/"

    mv pepper_dataset "${OLD_DIR}"

    if [ $? -eq 0 ]; then
        echo "   ✅ Backup successful!"
        echo "   Location: ${OLD_DIR}/"
    else
        echo "   ❌ Backup failed!"
        exit 1
    fi
else
    echo "📦 Step 1: No existing dataset found (first time)"
fi

echo ""

# Step 2: Create new dataset structure
echo "📁 Step 2: Creating new dataset structure..."

mkdir -p pepper_dataset

echo "   ✅ Created: pepper_dataset/"
echo ""

# Summary
echo "================================================================================"
echo "✅ SETUP COMPLETE!"
echo "================================================================================"
echo ""
echo "📊 Dataset Structure:"
echo "   pepper_dataset/              ← New (empty, ready for collection)"
if [ -d "${OLD_DIR}" ]; then
    echo "   ${OLD_DIR}/    ← Backup (old data)"
fi
echo ""
echo "🚀 Next Steps:"
echo "   1. Run collection script for each session"
echo "   2. Follow DATASET_RECOLLECTION_GUIDE.md"
echo ""
echo "📝 Sessions to collect:"
echo "   Session 1: Red large         (8 peppers × 12 angles = 96 images)"
echo "   Session 2: Red defects       (4 peppers × 12 angles = 48 images)"
echo "   Session 3: Green varieties   (4 peppers × 12 angles = 48 images)"
echo "   ─────────────────────────────────────────────────────────────"
echo "   Total:                       (16 peppers × 12 angles = 192 images)"
echo ""
echo "⏱️  Estimated time: ~60-75 minutes total"
echo ""
echo "================================================================================"
