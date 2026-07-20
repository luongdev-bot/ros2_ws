# ros2_ws — Project Instructions

> Định hướng chi tiết về kiến trúc, skills, commands và rules nằm ở
> `.claude/CLAUDE.md`. File này chỉ bổ sung quy trình làm việc.

## Quy trình code

Sau khi viết hoặc sửa code xong, LUÔN review các thay đổi chưa commit
TRƯỚC khi báo hoàn thành. Có hai reviewer; dùng Copilot làm mặc định.

### Mặc định — GitHub Copilot CLI

Copilot CLI không chạy được ở chế độ MCP server (`copilot mcp` chỉ để
quản lý MCP server của nó), nên gọi qua Bash:

```bash
copilot -p "Review the uncommitted changes in this repo. Run \`git diff\` \
and \`git diff --staged\` to see them. Report bugs, correctness issues, \
and design problems with file:line anchors. Do NOT modify any files." \
  --deny-tool='write' \
  --allow-tool='shell(git diff)' \
  --allow-tool='shell(git diff --staged)' \
  --allow-tool='shell(git status)' \
  --allow-tool='shell(git log)'
```

- `--deny-tool='write'` là ràng buộc chính khiến Copilot không sửa được
  code — không chỉ dựa vào câu "do NOT modify" trong prompt. Luôn giữ
  flag này; không dùng `--allow-all-tools` / `--yolo` cho bước review.
- Allowlist khớp lỏng (một chuỗi lệnh ghép `git a && git b` vẫn qua),
  nên đừng nới thành `shell(git)` — sẽ cho phép cả `git reset --hard`
  và `git push`.
- Tốn khoảng 6 AI credits mỗi lần review. Ưu tiên review ở mốc có ý
  nghĩa (xong một tính năng) hơn là sau mỗi sửa đổi nhỏ.

### Thay thế — Codex CLI

`codex review` là subcommand chuyên cho review, chạy non-interactive:

```bash
codex review --uncommitted
```

Biến thể: `--commit <SHA>` (một commit), `--base <BRANCH>` (diff so với
nhánh gốc), hoặc truyền chuỗi hướng dẫn riêng làm tham số.

> **Quota:** tài khoản Codex (gói Go) đã hết hạn mức ngày 20/07/2026,
> reset **26/07/2026 21:22**. Trước mốc đó lệnh sẽ báo
> `You've hit your usage limit` và review đứt giữa chừng — dùng Copilot.

### Chung cho cả hai

- Nếu reviewer chỉ ra lỗi hoặc góp ý hợp lý, tự sửa lại rồi review lần
  nữa cho đến khi không còn lỗi nghiêm trọng.
- Không dùng Copilot hay Codex để viết code — chỉ dùng để review.
- Nếu lệnh lỗi, hết quota, hoặc không phản hồi, báo cho tôi biết thay vì
  bỏ qua bước review.

Workspace này là git repo, nên `git diff` là nguồn sự thật cho "thay đổi
chưa commit". Nếu working tree clean thì không có gì để review.
