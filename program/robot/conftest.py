# Memastikan paket `mizuro` bisa diimpor saat pytest dijalankan dari folder
# ini (pytest hanya menambahkan direktori berkas tes ke sys.path).
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
