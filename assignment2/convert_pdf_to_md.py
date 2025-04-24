"""
Simple PDF-to-Markdown converter using PyPDF2.
Usage:
  1. Install PyPDF2: pip install PyPDF2
  2. Run: python convert_pdf_to_md.py assignment2.pdf assignment2.md
"""
import sys
from PyPDF2 import PdfReader

def pdf_to_md(pdf_path, md_path):
    reader = PdfReader(pdf_path)
    with open(md_path, 'w', encoding='utf-8') as f:
        for page in reader.pages:
            text = page.extract_text() or ''
            f.write(text)
            f.write("\n\n")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python convert_pdf_to_md.py <input.pdf> <output.md>")
        sys.exit(1)
    pdf_to_md(sys.argv[1], sys.argv[2])
