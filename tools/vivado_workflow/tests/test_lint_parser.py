"""Regression coverage for the Ross 2026.9.1 empty-report parsing defect."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from parse_lint_report import extract_synthesis_insights


def table(rows):
    headers = ['Rule ID', 'description', 'rtl_name', 'info',
               'rtl_hierarchy', 'rtl_file', 'Line number']
    widths = [max(len(row[i]) for row in [headers] + rows) + 2 for i in range(7)]
    border = '+' + '+'.join('-' * width for width in widths) + '+'
    def format_row(row):
        return '|' + '|'.join(' ' + cell.ljust(width - 1)
                              for cell, width in zip(row, widths)) + '|'
    result = ['2. Expanded', border, format_row(headers), border]
    result += [format_row(row) for row in rows]
    if rows:
        result.append(border)
    result.append('* create_waiver can be used to apply waiver to linter violations.')
    result.append('** For example, create_waiver -type LINT -id RULE_ID {-rtl_name RTL_NAME}.')
    return '\n'.join(result)


class LintParserTests(unittest.TestCase):
    def parse(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'linter.rpt'
            path.write_text(text)
            return extract_synthesis_insights(path)

    def test_empty_table_footer_is_not_a_violation(self):
        self.assertEqual(self.parse(table([])), '')

    def test_real_violation_with_literal_pipe_is_preserved(self):
        row = ['ASSIGN-2', 'operator example', 'a|b', 'N/A', 'led_blink', 'led_blink.sv', '17']
        self.assertEqual(self.parse(table([row])), ','.join(row))


if __name__ == '__main__':
    unittest.main()
