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

### Chạy Codex — luôn mở terminal mới, KHÔNG chạy ngầm

Mọi lần gọi Codex (giao việc lẫn review) **phải chạy trong một cửa sổ
`gnome-terminal` mới**, bằng lệnh CLI `codex`. Tuyệt đối không chạy ngầm:
không dùng MCP tool `mcp__codex-cli__codex`, không `run_in_background`.

**Lý do:** mỗi lần chạy tạo một session hiển thị được và resume được, lưu
ở `~/.codex/sessions/YYYY/MM/DD/rollout-*.jsonl`. Sau này khi cần đưa ra
lịch sử trò chuyện với Codex (ví dụ người phỏng vấn yêu cầu), mở lại bằng
`codex resume --last` (hoặc `codex exec resume --last`), hoặc nộp thẳng
file `.jsonl` đó. Chạy ngầm không để lại lịch sử xem được.

Mẫu bọc lệnh — cửa sổ tự giữ mở (`exec bash`) để xem và resume; output
vừa hiện trên màn hình vừa ghi ra `/tmp` để Claude đọc lại:

```bash
gnome-terminal --title="codex: <việc>" -- bash -lc '
cd <thư mục làm việc>
<lệnh codex đầy đủ> 2>&1 | tee /tmp/codex-<việc>.log
echo "[[CODEX_DONE]]" >> /tmp/codex-<việc>.log
exec bash'
```

`gnome-terminal` trả về ngay và cửa sổ chạy độc lập, nên **Claude không
tự nhận được output**. Claude phải đợi tới khi dòng `[[CODEX_DONE]]` xuất
hiện trong `/tmp/codex-<việc>.log` rồi mới đọc file đó. Chưa thấy dấu này
nghĩa là Codex còn chạy hoặc cửa sổ lỗi — không phải sạch lỗi, và không
được báo hoàn thành.

Prompt đặt trong ngoặc kép của lệnh `codex`; tránh dấu nháy đơn `'` trong
prompt vì nó cắt chuỗi single-quote của `bash -lc`.

**Hai cạm bẫy đã gặp thật khi dùng mẫu này với `codex review`:**

- `codex review | tee` không cho findings gọn — khi stdout không phải
  TTY, nó đổ ra JSONL session thô, rất khó bới lại khối nhận xét. Đọc
  findings ngay trong **cửa sổ** (đó là TTY, hiện đẹp), hoặc bọc bằng
  `script -q -c '<lệnh codex review>' /tmp/codex-<việc>.log` để vừa giữ
  TTY vừa lưu file. `codex exec` thì `| tee` bình thường.
- **Thoát mà không có findings ≠ sạch lỗi.** Nếu diff review đụng tới
  chủ đề file hệ thống (ví dụ chính `~/.codex/sessions/*.jsonl`), review
  agent hay bò sang đọc mớ đó rồi hết budget, thoát mà chưa kịp nhận
  xét — `[[CODEX_DONE]]` vẫn nổ. Phải xác nhận log THỰC SỰ có khối
  findings; nếu không thì coi là review lỗi, chạy lại và nêu rõ trong
  prompt "chỉ đọc diff, không đọc file ngoài repo".

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

Giao việc bằng `codex exec` (non-interactive, tự chạy) trong terminal mới
theo đúng mẫu bọc lệnh ở trên:

```bash
gnome-terminal --title="codex: fix-detector" -- bash -lc '
cd /home/luong/ros2_ws
codex exec --sandbox workspace-write -m gpt-5.6-sol \
  -c model_reasoning_effort=xhigh \
  "<mô tả nhiệm vụ thật cụ thể: sửa file nào, hành vi mong muốn, ràng buộc kiến trúc, test phải qua>" \
  2>&1 | tee /tmp/codex-fix-detector.log
echo "[[CODEX_DONE]]" >> /tmp/codex-fix-detector.log
exec bash'
```

Đổi `cd /home/luong/ros2_ws` sang worktree ở bước 1 nếu có. `codex exec`
chạy autonomous, không có bước xác nhận; nếu nó khựng lại thì thấy ngay
trong cửa sổ.

`--sandbox workspace-write` cho Codex quyền **sửa và xoá file thật**.
Không có cấu hình nào ngăn được Codex đụng file ngoài phạm vi. Thứ thật
sự giới hạn thiệt hại là **chọn đúng thư mục làm việc và đúng sandbox**;
git chỉ giúp *phát hiện và khôi phục* thay đổi trên file đã track — nó
không cứu được file bị ignore hay file ngoài repo. Điều kiện để được
giao việc: **worktree riêng** nếu việc đó có thể đụng file bị ignore
(build artifact, `install/`, config cục bộ); chỉ khi chắc chắn không
đụng thì mới được giao trên tree sạch. Luôn dùng `--sandbox read-only`
khi review.

Chọn model theo việc: `gpt-5.6-sol` cho task thường. Mặc định trong
`~/.codex/config.toml` chỉ áp dụng khi không truyền `model`.

**Luôn để Codex suy nghĩ ở mức cao nhất — `xhigh`.** Thêm
`-c model_reasoning_effort=xhigh` vào mọi lệnh `codex` (cả `exec` lẫn
`review`). Ghi cứng ở đây để không phụ thuộc vào `config.toml` — file đó
có thể bị đổi, và `codex review` còn dùng mặc định riêng (`high`) thấp
hơn. Header phiên chạy sẽ in `reasoning effort: xhigh` để kiểm.

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

Claude tự đánh giá diff trước. Cần ý kiến thứ hai thì chạy `codex review`
trong terminal mới (cũng theo mẫu bọc lệnh ở trên):

```bash
gnome-terminal --title="codex review" -- bash -lc '
cd /home/luong/ros2_ws
codex review --uncommitted -c model_reasoning_effort=xhigh \
  2>&1 | tee /tmp/codex-review.log
echo "[[CODEX_DONE]]" >> /tmp/codex-review.log
exec bash'
```

Các biến thể của `codex review` (thay `--uncommitted` trong lệnh trên):

```bash
codex review --uncommitted        # thay đổi chưa commit
codex review --commit 66033de     # đúng một commit
codex review --base master        # diff so với nhánh gốc (repo này: master)
codex review 'Focus on bugs'      # prompt tự do, KHÔNG kèm flag định phạm vi
```

(luôn kèm `-c model_reasoning_effort=xhigh` như trong lệnh bọc trên.)

Ba flag phạm vi **xung khắc với prompt vị trí** — `codex review
--uncommitted 'Focus...'` báo `cannot be used with [PROMPT]`. Muốn hướng
dẫn riêng thì để `codex review 'prompt'` đứng một mình.

Hỏi tiếp trong cùng phiên review bằng `codex resume --last` trong chính
cửa sổ đó, thay vì mở phiên mới.

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

**Đợi đúng dấu `[[CODEX_DONE]]` rồi mới đọc log.** Cửa sổ chạy độc lập
nên Claude không tự biết Codex xong lúc nào; chưa thấy dấu này thì Codex
còn chạy hoặc cửa sổ lỗi — không phải sạch lỗi. Nếu log cho thấy lệnh
lỗi, hết quota, hay cửa sổ không mở được thì đó là review thất bại thật:
phải báo người dùng và **không được** coi như đã review xong.

Các agent trong `.claude/CLAUDE.md` (`ros2-style-reviewer`,
`gz-style-reviewer`, …) là công cụ riêng cho review trước khi mở PR —
bao quát cả lifecycle, QoS, pluginlib, test và build manifest. Dùng thêm
khi cần, không thay thế các bước trên.
