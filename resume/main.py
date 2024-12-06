from docx import Document

doc = Document()

# 3x3 크기의 표 추가
table = doc.add_table(rows=3, cols=3)
table.style = 'Table Grid' # 표의 각 셀에 격자선 추가

# 첫 번째 행의 셀들을 hdr_cells 변수에 할당하고 각 셀에 헤더 데이터 추가
hdr_cells = table.rows[0].cells 
hdr_cells[0].text = '헤더 1' 
hdr_cells[1].text = '헤더 2'
hdr_cells[2].text = '헤더 3'

# for문으로 각 행의 셀에 접근하여 두 번째와 세 번째 행에 데이터 추가
for i in range(1, 3):
    row_cells = table.rows[i].cells
    row_cells[0].text = f'행 {i}, 열 1'
    row_cells[1].text = f'행 {i}, 열 2'
    row_cells[2].text = f'행 {i}, 열 3'

doc.save('example_table.docx')