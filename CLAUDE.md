# ros2_ws — Project Instructions

> Định hướng chi tiết về kiến trúc, skills, commands và rules nằm ở
> `.claude/CLAUDE.md`. File này chỉ bổ sung quy trình làm việc.

## Quy trình code

### Vai trò

**Claude không viết code.** Claude đọc hiểu yêu cầu của người dùng, chia
việc, ra lệnh cho Codex, rồi chấm điểm và kiểm tra lại code Codex viết
xong. Codex là người viết toàn bộ code.

**Ngoại lệ duy nhất:** khi Codex hết quota / hết token và không chạy
được nữa, Claude mới trực tiếp viết code — và phải nói rõ với người dùng
là đang viết thay vì giao, kèm lý do.

Ngay cả khi tự viết, bước đọc lại và review ở dưới vẫn áp dụng.

### Bước 1 — Chuẩn bị chỗ làm việc

Cần một mốc git sạch để tách được thay đổi của Codex ra khỏi phần khác.

**Không tự ý `git commit` hay `git stash` công việc đang dở của người
dùng** — đó là trạng thái thuộc về họ, và `stash` không giữ file bị
ignore. Nếu tree đang bẩn thì hỏi trước.

**Ghi lại SHA mốc trước khi giao việc** — Codex có quyền `git commit`,
và nếu nó commit thì `git diff` sẽ trắng trơn dù đã sửa rất nhiều:

```bash
git rev-parse HEAD    # ghi lại giá trị này
```

Mỗi lệnh Bash chạy trong shell riêng nên biến môi trường không sống sót
giữa các lần gọi — **chép lấy SHA thật và thay vào các lệnh ở bước 3**,
đừng dựa vào một biến `BASE`.

**Việc có thể đụng tới file bị ignore thì bắt buộc dùng worktree riêng** —
build artifact, `install/`, config máy cục bộ. Tree sạch theo git không
nói lên được gì về những file đó. Dùng worktree riêng, không phải nhánh
mới:

```bash
git worktree add -b fix-detector /tmp/wt-fix-detector
```

(dùng tên thật; đừng gõ dấu ngoặc nhọn vì shell coi `<` `>` là chuyển
hướng.)

Nhánh mới trong cùng thư mục **không cách ly gì cả** — vẫn chung working
tree và index, nên `workspace-write` vẫn chạm được file đang dở của
người dùng. Đổi lại, worktree riêng cần build lại từ đầu — đó là cái giá
phải trả khi việc giao có thể đụng file bị ignore, không phải thứ để cân
nhắc bỏ qua.

### Bước 2 — Giao việc cho Codex

```yaml
tool: mcp__codex-cli__codex
prompt: >-
  <mô tả nhiệm vụ thật cụ thể: sửa file nào, hành vi mong muốn,
  ràng buộc kiến trúc, test phải qua>
cwd: <thư mục đang làm việc thật>   # ros2_ws, hoặc worktree ở bước 1
model: gpt-5.6-sol
sandbox: workspace-write
approval-policy: never
config: { model_reasoning_effort: xhigh }
```

`sandbox: workspace-write` cho Codex quyền **sửa và xoá file thật**, và
`approval-policy: never` nghĩa là không có bước xác nhận nào — kể cả
`on-request` cũng do model tự quyết lúc nào hỏi, không phải bảo đảm.
Không có cấu hình nào ngăn được Codex đụng file ngoài phạm vi. Thứ thật
sự giới hạn thiệt hại là **chọn đúng thư mục làm việc và đúng sandbox**;
git chỉ giúp *phát hiện và khôi phục* thay đổi trên file đã track — nó
không cứu được file bị ignore hay file ngoài repo. Điều kiện để được
giao việc: **worktree riêng** nếu việc đó có thể đụng file bị ignore
(build artifact, `install/`, config cục bộ); chỉ khi chắc chắn không
đụng thì mới được giao trên tree sạch. Luôn quay về `read-only` khi
review.

Chọn model theo việc: `gpt-5.6-sol` cho task thường. Mặc định trong
`~/.codex/config.toml` chỉ áp dụng khi không truyền `model`.

**Luôn để Codex suy nghĩ ở mức cao nhất — `xhigh`.** Truyền
`config: { model_reasoning_effort: xhigh }` trong mọi lời gọi MCP (cả
giao việc lẫn review), và `-c model_reasoning_effort=xhigh` cho lệnh CLI.
Ghi cứng ở đây để không phụ thuộc vào `config.toml` — file đó có thể bị
đổi, và `codex review` còn dùng mặc định riêng (`high`) thấp hơn.

### Bước 3 — Đọc lại những gì Codex đã thay đổi

Bắt buộc, không được bỏ:

```bash
git log --oneline 45bfccc..HEAD             # Codex có tự commit không
git diff 45bfccc..HEAD                      # nội dung các commit đó
git status --short                          # file nào bị đụng
git diff                                    # thay đổi chưa stage
git diff --cached                           # thay đổi đã stage
git ls-files --others --exclude-standard    # file mới chưa track
```

(thay `45bfccc` bằng SHA đã ghi ở bước 1)

`git diff` **không** bao gồm phần đã stage, file mới, lẫn thứ đã được
commit — thiếu các lệnh còn lại là thay đổi của Codex sẽ lọt qua mà
không ai đọc. Với file mới, phải mở ra đọc nội dung chứ không chỉ nhìn
tên.

**File bị ignore là điểm mù của toàn bộ các lệnh trên.** Codex vẫn xoá
hay ghi đè được `build/`, `install/`, hay config máy cục bộ mà không
lệnh nào ở đây phát hiện, và git cũng không khôi phục được. Đây chính là
lý do việc có khả năng đụng tới chúng phải giao trong worktree riêng —
không phụ thuộc vào việc ta cho rằng chúng có quan trọng hay không.

**Đọc diff thật, đừng tin bản tóm tắt của Codex.** Nó báo đã làm gì,
không đảm bảo khớp với thứ nằm trên đĩa. Cụ thể phải soi: file bị sửa
ngoài phạm vi giao việc, test bị xoá hay bị nới lỏng cho dễ qua, và phụ
thuộc mới thêm vào.

Kiểm tra ranh giới kiến trúc theo đúng loại code (xem `.claude/CLAUDE.md`):
package ROS 2 thì `domain/` không được import `rclpy` / `*_msgs`; code
gz-sim thì theo quy ước ECS — dữ liệu nằm trong component, hành vi nằm
trong system.

### Bước 4 — Chấm điểm và review

Claude tự đánh giá diff trước. Cần ý kiến thứ hai thì gọi Codex ở chế độ
read-only:

```yaml
tool: mcp__codex-cli__codex
prompt: >-
  Review all uncommitted changes. Inspect `git status --short`,
  `git diff`, `git diff --cached`, and any untracked files. Report
  bugs and design problems with file:line anchors.
  Do not modify any files.
cwd: <cùng thư mục đã giao việc>     # phải khớp bước 2
sandbox: read-only
approval-policy: never
config: { model_reasoning_effort: xhigh }
```

Tool trả kèm `threadId`; dùng `codex-reply` với threadId đó để hỏi tiếp
trong cùng phiên thay vì mở phiên mới.

Hoặc dùng CLI:

```bash
# thêm -c model_reasoning_effort=xhigh vào mọi lệnh dưới đây
codex review --uncommitted -c model_reasoning_effort=xhigh   # thay đổi chưa commit
codex review --commit 66033de -c model_reasoning_effort=xhigh # đúng một commit
codex review --base master -c model_reasoning_effort=xhigh    # diff so với nhánh gốc (repo này: master)
codex review -c model_reasoning_effort=xhigh 'Focus on bugs'  # prompt tự do, KHÔNG kèm flag định phạm vi
```

Ba flag trên **xung khắc với prompt vị trí** — `codex review --uncommitted
'...'` báo `cannot be used with [PROMPT]`. Muốn hướng dẫn riêng thì dùng
MCP tool, hoặc `codex review` với prompt đứng một mình.

### Xem lại lịch sử trò chuyện với Codex

Mỗi lần gọi Codex (qua MCP tool hay CLI) đều tự lưu một session ở
`~/.codex/sessions/YYYY/MM/DD/rollout-*.jsonl` — đó là lịch sử chat đầy
đủ (bên giao việc ↔ Codex). Mở lại dưới dạng hội thoại khi cần trình ra:

```bash
codex resume --last          # mở lại phiên gần nhất trong giao diện chat
codex resume <session-id>     # hoặc theo id (UUID hay tên phiên)
```

Không cần dựng terminal riêng để có lịch sử này — MCP tool chạy ngầm vẫn
tạo đúng session đó; `codex resume` hiển thị nó như một khung hội thoại.

### Vòng lặp bắt buộc

Review là **điều kiện để được báo hoàn thành**, không phải bước tùy chọn:

1. Codex viết xong → Claude đọc lại toàn bộ thay đổi (bước 3).
2. Review (bước 4). Lỗi hợp lý thì giao Codex sửa, rồi **review lại**.
3. Lặp cho đến khi không còn lỗi nghiêm trọng.
4. Chỉ khi đó mới báo hoàn thành.

Sau khi Codex sửa, `--commit <SHA cũ>` vẫn review commit cũ chứ không
thấy phần sửa — dùng `--uncommitted`, hoặc commit/amend rồi truyền SHA
mới.

Nếu lệnh lỗi, hết quota, hoặc không phản hồi thì **báo cho người dùng
biết**, tuyệt đối không lặng lẽ bỏ qua bước review rồi báo xong.

### Quy tắc chung

**Chạy từ trong git worktree.** Ngoài worktree, `codex review` dừng ngay
với `Not inside a trusted directory and --skip-git-repo-check was not
specified` — lỗi rõ ràng, không phải treo hay chạy sai.

**Giữ diff nhỏ.** Một package mới ~2000 dòng chạy hết ~9 phút và dễ
timeout; diff vài chục dòng chỉ mất chưa tới một phút. Chia việc theo
từng phần thay vì giao trọn gói.

**Ghi output CLI vào `/tmp`, không ghi trong worktree** — `--uncommitted`
tính cả file chưa track, nên Codex sẽ review chính file output của nó.

**Timeout không phải im lặng.** Codex đưa nhận xét sau khi đọc hết diff;
lệnh bị cắt giữa chừng nghĩa là không có nhận xét nào, đừng hiểu nhầm là
sạch lỗi.

**Codex chạy trong sandbox riêng**, không thấy cấu hình MCP của Claude
Code. Nếu bản thân lượt review chạy được nhưng *trong nội dung nhận xét*
Codex nói server `codex-cli` không tồn tại hay không kết nối được, thì
đó chỉ là chẩn đoán sai của nó — bỏ qua nhận xét đó, muốn chắc thì tự
chạy `claude mcp list`. Ngược lại, nếu chính lời gọi review thất bại thì
đó là review thất bại thật: phải báo người dùng và **không được** coi
như đã review xong.

Các agent trong `.claude/CLAUDE.md` (`ros2-style-reviewer`,
`gz-style-reviewer`, …) là công cụ riêng cho review trước khi mở PR —
bao quát cả lifecycle, QoS, pluginlib, test và build manifest. Dùng thêm
khi cần, không thay thế các bước trên.
