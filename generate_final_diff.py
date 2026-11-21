import difflib

file1 = r'C:\Users\USER\Desktop\Memo_Arduino\final_deadline_candidate\final_deadline_candidate.ino'
file2 = r'C:\Users\USER\Desktop\MEMO_ESP32_FINAL\MEMO_ESP32_FINAL.ino'
out_patch = r'C:\Users\USER\Desktop\Memo_Arduino\diff_deadline_to_final.patch'

with open(file1, 'r', encoding='utf-8') as f:
    lines1 = f.readlines()

with open(file2, 'r', encoding='utf-8') as f:
    lines2 = f.readlines()

diff = list(difflib.unified_diff(
    lines1, lines2,
    fromfile='final_deadline_candidate.ino',
    tofile='MEMO_ESP32_FINAL.ino',
    lineterm='\n'
))

with open(out_patch, 'w', encoding='utf-8') as f:
    f.writelines(diff)

print("Unified diff generated successfully at:", out_patch)
print("Diff lines count:", len(diff))
