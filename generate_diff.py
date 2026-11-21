import difflib

file1 = r'C:\Users\USER\Desktop\Memo_Arduino\final_audio_stability_v2_3\final_audio_stability_v2_3.ino'
file2 = r'C:\Users\USER\Desktop\Memo_Arduino\final_deadline_candidate\final_deadline_candidate.ino'
out_patch = r'C:\Users\USER\Desktop\Memo_Arduino\diff_v2_3_to_deadline.patch'

with open(file1, 'r', encoding='utf-8') as f:
    lines1 = f.readlines()

with open(file2, 'r', encoding='utf-8') as f:
    lines2 = f.readlines()

diff = list(difflib.unified_diff(
    lines1, lines2,
    fromfile='final_audio_stability_v2_3.ino',
    tofile='final_deadline_candidate.ino',
    lineterm='\n'
))

with open(out_patch, 'w', encoding='utf-8') as f:
    f.writelines(diff)

print("Unified diff generated successfully at:", out_patch)
print("Diff lines count:", len(diff))
