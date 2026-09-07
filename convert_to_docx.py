#!/usr/bin/env python3
"""Convert TDD markdown to .docx with proper formatting."""

from docx import Document
from docx.shared import Pt, Inches, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
import re
import os

def set_cell_shading(cell, color):
    """Set cell background color."""
    shading_elm = cell._element.get_or_add_tcPr()
    shading = shading_elm.makeelement(qn('w:shd'), {
        qn('w:fill'): color,
        qn('w:val'): 'clear',
    })
    shading_elm.append(shading)

def add_code_block(doc, code_text, font_size=8.5):
    """Add a code block with monospace font and light background."""
    for line in code_text.strip().split('\n'):
        p = doc.add_paragraph()
        p.paragraph_format.space_before = Pt(0)
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.line_spacing = Pt(13)
        run = p.add_run(line)
        run.font.name = 'Consolas'
        run.font.size = Pt(font_size)
        run.font.color.rgb = RGBColor(0x1a, 0x1a, 0x1a)
    # Add a small gap after code block
    doc.add_paragraph().paragraph_format.space_before = Pt(2)

def add_styled_paragraph(doc, text, style=None, bold=False, font_size=None, color=None, alignment=None):
    """Add a paragraph with optional styling."""
    p = doc.add_paragraph(style=style)
    if alignment is not None:
        p.alignment = alignment
    run = p.add_run(text)
    if bold:
        run.bold = True
    if font_size:
        run.font.size = Pt(font_size)
    if color:
        run.font.color.rgb = color
    return p

def parse_inline_markup(doc, paragraph, text):
    """Parse inline bold, code, and plain text in a single line."""
    # Pattern: **bold** or `code` or plain text
    pattern = re.compile(r'(\*\*(.+?)\*\*|`(.+?)`|(.+?))')
    parts = pattern.findall(text)
    for part in parts:
        full, bold_text, code_text, plain = part
        if bold_text:
            run = paragraph.add_run(bold_text)
            run.bold = True
        elif code_text:
            run = paragraph.add_run(code_text)
            run.font.name = 'Consolas'
            run.font.size = Pt(9)
        elif plain:
            paragraph.add_run(plain)

def convert_markdown_to_docx(md_path, docx_path):
    doc = Document()

    # Set default font
    style = doc.styles['Normal']
    font = style.font
    font.name = '微软雅黑'
    font.size = Pt(10.5)
    style.paragraph_format.space_after = Pt(6)
    style.paragraph_format.line_spacing = 1.25

    # Set page margins
    for section in doc.sections:
        section.top_margin = Cm(2.0)
        section.bottom_margin = Cm(2.0)
        section.left_margin = Cm(2.5)
        section.right_margin = Cm(2.0)

    with open(md_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    i = 0
    in_code_block = False
    code_lines = []
    in_table = False
    table_lines = []
    in_ascii_art = False
    ascii_lines = []

    while i < len(lines):
        line = lines[i].rstrip()

        # Handle code blocks
        if line.startswith('```'):
            if in_code_block:
                add_code_block(doc, '\n'.join(code_lines))
                code_lines = []
                in_code_block = False
            else:
                in_code_block = True
            i += 1
            continue

        if in_code_block:
            code_lines.append(line)
            i += 1
            continue

        # Handle table detection
        if '|' in line and line.strip().startswith('|') and line.strip().endswith('|'):
            if not in_table:
                in_table = True
                table_lines = []
            table_lines.append(line)
            # Check if next line is still part of table
            if i + 1 < len(lines) and '|' in lines[i + 1] and lines[i + 1].strip().startswith('|'):
                i += 1
                continue
            else:
                # Process accumulated table
                process_table(doc, table_lines)
                table_lines = []
                in_table = False
                i += 1
                continue
        elif in_table:
            # Flush table if line doesn't belong
            process_table(doc, table_lines)
            table_lines = []
            in_table = False

        # Detect ASCII art diagrams (indented blocks with special chars)
        stripped = line.strip()
        if stripped and (stripped.startswith('┌') or stripped.startswith('│') or
                         stripped.startswith('└') or stripped.startswith('├') or
                         stripped.startswith('▼') or stripped.startswith('▶') or
                         stripped.startswith('◄') or '──' in stripped or
                         '↓' in stripped or '→' in stripped):
            if not in_ascii_art:
                in_ascii_art = True
                ascii_lines = []
            ascii_lines.append(line)
            i += 1
            continue
        elif in_ascii_art:
            # End of ASCII art block
            add_code_block(doc, '\n'.join(ascii_lines), font_size=7)
            ascii_lines = []
            in_ascii_art = False

        # Horizontal rule
        if line.strip() == '---':
            p = doc.add_paragraph()
            p.paragraph_format.space_before = Pt(6)
            p.paragraph_format.space_after = Pt(6)
            pPr = p._p.get_or_add_pPr()
            pBdr = pPr.makeelement(qn('w:pBdr'), {})
            bottom = pBdr.makeelement(qn('w:bottom'), {
                qn('w:val'): 'single',
                qn('w:sz'): '6',
                qn('w:space'): '4',
                qn('w:color'): '999999',
            })
            pBdr.append(bottom)
            pPr.append(pBdr)
            i += 1
            continue

        # Headings
        if line.startswith('# '):
            p = doc.add_heading(line[2:].strip(), level=1)
            i += 1
            continue
        if line.startswith('## '):
            p = doc.add_heading(line[3:].strip(), level=2)
            i += 1
            continue
        if line.startswith('### '):
            p = doc.add_heading(line[4:].strip(), level=3)
            i += 1
            continue
        if line.startswith('#### '):
            p = doc.add_heading(line[5:].strip(), level=4)
            i += 1
            continue

        # Bold text lines (standalone, not inline)
        if line.startswith('**') and line.endswith('**') and not line.startswith('**`'):
            p = doc.add_paragraph()
            run = p.add_run(line[2:-2])
            run.bold = True
            i += 1
            continue

        # List items
        if re.match(r'^- ', line):
            text = re.sub(r'^- ', '', line)
            p = doc.add_paragraph(style='List Bullet')
            # Clear default text and add parsed inline markup
            p.clear()
            parse_inline_markup(doc, p, text)
            i += 1
            continue

        if re.match(r'^\d+\.\s', line):
            text = re.sub(r'^\d+\.\s', '', line)
            p = doc.add_paragraph(style='List Number')
            p.clear()
            parse_inline_markup(doc, p, text)
            i += 1
            continue

        # Empty line
        if not line.strip():
            # Skip adding empty paragraphs - python-docx handles spacing
            i += 1
            continue

        # Regular paragraph with inline markup
        p = doc.add_paragraph()
        parse_inline_markup(doc, p, line)
        i += 1

    # Flush any remaining ASCII art
    if in_ascii_art:
        add_code_block(doc, '\n'.join(ascii_lines), font_size=7)

    # Flush any remaining code block
    if in_code_block:
        add_code_block(doc, '\n'.join(code_lines))

    doc.save(docx_path)
    print(f"Document saved to: {docx_path}")

def process_table(doc, table_lines):
    """Process a markdown table and add it to the document."""
    if len(table_lines) < 2:
        return

    # Parse rows
    rows = []
    for line in table_lines:
        cells = [c.strip() for c in line.strip().split('|')[1:-1]]
        rows.append(cells)

    # Filter out separator rows (e.g., |---|---|)
    data_rows = []
    header_row = None
    for row in rows:
        if all(re.match(r'^[-:]+$', c) for c in row):
            # This is a separator row, the row before it is the header
            if data_rows:
                header_row = data_rows.pop()
            continue
        data_rows.append(row)

    if not data_rows:
        return

    num_cols = max(len(r) for r in data_rows)
    if header_row is None:
        header_row = data_rows.pop(0)

    # Pad header row
    while len(header_row) < num_cols:
        header_row.append('')

    table = doc.add_table(rows=1 + len(data_rows), cols=num_cols, style='Table Grid')
    table.alignment = WD_TABLE_ALIGNMENT.CENTER

    # Header row
    for j, cell_text in enumerate(header_row):
        cell = table.rows[0].cells[j]
        cell.text = ''
        p = cell.paragraphs[0]
        run = p.add_run(cell_text)
        run.bold = True
        run.font.size = Pt(9.5)
        run.font.name = '微软雅黑'
        p.paragraph_format.space_before = Pt(2)
        p.paragraph_format.space_after = Pt(2)
        set_cell_shading(cell, 'D9E2F3')

    # Data rows
    for i, row in enumerate(data_rows):
        while len(row) < num_cols:
            row.append('')
        for j, cell_text in enumerate(row):
            cell = table.rows[i + 1].cells[j]
            cell.text = ''
            p = cell.paragraphs[0]
            run = p.add_run(cell_text)
            run.font.size = Pt(9.5)
            run.font.name = '微软雅黑'
            p.paragraph_format.space_before = Pt(1)
            p.paragraph_format.space_after = Pt(1)

    # Add spacing after table
    doc.add_paragraph().paragraph_format.space_before = Pt(2)

if __name__ == '__main__':
    md_path = r'd:\个人资料\工作\翔天飞宇\arm\基于ROS2的工业机械臂任务执行框架设计方案_V1.0.md'
    docx_path = r'd:\个人资料\工作\翔天飞宇\arm\基于ROS2的工业机械臂任务执行框架设计方案_V1.0.docx'
    convert_markdown_to_docx(md_path, docx_path)
