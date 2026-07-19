# ros2_ws — Project Instructions

> Định hướng chi tiết về kiến trúc, skills, commands và rules nằm ở
> `.claude/CLAUDE.md`. File này chỉ bổ sung quy trình làm việc.

## Quy trình code

- Sau khi viết hoặc sửa code xong, LUÔN gọi tool `codex` của MCP server
  `codex-cli` (tên đầy đủ: `mcp__codex-cli__codex`) để review lại các
  thay đổi chưa commit, TRƯỚC khi báo hoàn thành.
- Prompt gửi cho tool `codex` phải yêu cầu rõ: chỉ review, không sửa
  code. Ví dụ:
  "Review the uncommitted changes in this repo (`git diff` and
  `git diff --staged`). Report bugs, correctness issues, and design
  problems with file:line anchors. Do NOT modify any files."
- Nếu Codex chỉ ra lỗi hoặc góp ý hợp lý, tự sửa lại rồi review
  lần nữa cho đến khi không còn lỗi nghiêm trọng.
- Không dùng codex để viết code — chỉ dùng để review. Codex có quyền
  ghi file, nên phải nói rõ "do not modify files" trong mọi prompt.
- Dùng `codex-reply` (kèm thread id) để hỏi tiếp trong cùng một phiên
  review thay vì mở phiên mới.
- Nếu tool lỗi hoặc không phản hồi, báo cho tôi biết thay vì bỏ qua
  bước review.
