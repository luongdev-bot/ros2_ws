# ros2_ws — Project Instructions

> Định hướng chi tiết về kiến trúc, skills, commands và rules nằm ở
> `.claude/CLAUDE.md`. File này chỉ bổ sung quy trình làm việc.

## Quy trình code

Reviewer là **Codex**. Sau khi viết hoặc sửa code xong, LUÔN review
TRƯỚC khi báo hoàn thành.

### Cách gọi — MCP tool (mặc định)

Gọi tool `codex` của MCP server `codex-cli` đã đăng ký. Đây là tool
prompt-based, không phải subcommand `codex review` của CLI:

```yaml
tool: mcp__codex-cli__codex
prompt: >-
  Review all uncommitted changes. Inspect `git status --short`,
  `git diff`, `git diff --cached`, and any untracked files. Report
  bugs and design problems with file:line anchors.
  Do not modify any files.
cwd: /home/luong/ros2_ws
sandbox: read-only
approval-policy: never
```

`sandbox: read-only` là ràng buộc thật khiến Codex không sửa được code —
không chỉ dựa vào câu "do not modify" trong prompt. Luôn giữ.

Tool trả kết quả có cấu trúc kèm `threadId`; dùng `codex-reply` với
threadId đó để hỏi tiếp trong cùng phiên thay vì mở phiên mới.

### Cách gọi — CLI (thay thế)

```bash
codex review --uncommitted                   # thay đổi chưa commit
codex review --commit 66033de                # đúng một commit
codex review --base main                     # diff so với nhánh gốc
codex review --uncommitted 'Focus on bugs'   # thêm hướng dẫn riêng
```

Chuỗi hướng dẫn là tham số vị trí; đặt trong **ngoặc đơn** để shell
không diễn giải backtick, `$(...)` hay biến. Thay `66033de` / `main`
bằng giá trị thật — đừng gõ dấu ngoặc nhọn vì shell coi `<` `>` là
chuyển hướng.

Nếu ghi output ra file, ghi vào `/tmp` chứ **đừng ghi trong worktree**:
`--uncommitted` bao gồm cả file chưa track, nên Codex sẽ review chính
file output đang lớn dần của nó.

### Quy tắc chung

**Chạy từ trong git worktree.** Thư mục con nào của repo cũng được; chỉ
khi chạy ngoài worktree thì lệnh git bên trong mới fail exit 128 và
Codex trả output vô nghĩa thay vì báo lỗi rõ.

**Giữ diff nhỏ.** Review trên diff 50+ file mất nhiều phút và thường
timeout. Commit theo từng chủ đề rồi review từng phần. Codex đưa nhận
xét sau khi đọc hết diff, nên timeout giữa chừng đồng nghĩa không có
nhận xét nào — đó là timeout, đừng nhầm là Codex im lặng.

**Vòng lặp sửa — review lại.** Sau khi sửa theo góp ý, `--commit <SHA cũ>`
vẫn review commit cũ chứ không thấy phần sửa. Dùng `--uncommitted` cho
phần vừa sửa, hoặc commit/amend rồi truyền SHA mới.

- Nếu Codex chỉ ra lỗi hoặc góp ý hợp lý, tự sửa lại rồi review lần nữa
  cho đến khi không còn lỗi nghiêm trọng.
- Không dùng Codex để viết code — chỉ dùng để review. Không chạy
  `codex exec` hay `codex apply` cho mục đích này.
- Nếu lệnh lỗi, hết quota, hoặc không phản hồi, báo cho tôi biết thay vì
  bỏ qua bước review.

Bước này là bắt buộc trước khi báo hoàn thành. Agent `ros2-style-reviewer`
trong `.claude/CLAUDE.md` là công cụ riêng cho review kiến trúc trước khi
mở PR — dùng thêm khi cần, không thay thế bước trên.
