# State.msg
# ミッションの状態のメッセージ定義
#   [topic]
#     /mission/state
#   [msg]
#     header:
#       seq: xxx # 一意な単調増加番号（自動採番）
#       stamp:
#         secs: xxxxxxxxxx # unix timeの秒
#         nsecs: xxxxxxxx # ナノ秒（精度はOS依存）
#       frame_id: mission_state
#     current: x # 現在のミッション番号
#     total: x # 合計のミッション番号
#     status: 0 # 初期状態
#             1 # ミッション実行中
#             2 # ミッション完了
#             3 # ミッション中断
#
# 状態遷移
# ●
# +
# |                                                           receive command: 1
# |                                                           (restart current mission)
# |                                                   +-----------------------------------------------------------------------------------+
# |                                                   |       receive command: 3                                                          |
# |                                                   |       (resume suspended mission)                                                  |
# |                                                   | +----------------------------------------------------------------------------+    |
# |                                                   | |     receive command: 2                                                     |    |
# |                                                   | |     (suspend current mission)                                              |    |
# |                                                   | | +---------------------------------------------------------------------+    |    |
# |                                                   | | |                                                                     |    |    |
# |                    receive command: 1             v v |                                                                     v    |    |
# |    +------------+  (start a new mission)     +----+-+-+---+                                  +------------+                ++----+----+-+
# |    | status: 0  +--------------------------->+ status: 1  |      arrive at the goal          | status: 2  |                | status: 3  |
# +--->+ (init)     |                            | (running)  +--------------------------------->+ (finish)   |                | (suspend)  |
#      +------------+                            +------------+                                  +------------+                +------------+
#      |            +<---------------------------+ do/move    +<---------------------------------+            |                |            |
#      +--------+--++  receive command: 0        +------------+      receive command: 1          +-----+------+                +-----+------+
#               ^  ^   (stop current mission)                        (start a new mission)             |                             |
#               |  |                                                                                   |                             |
#               |  +-----------------------------------------------------------------------------------+                             |
#               |      receive command: 0                                                                                            |
#               |      (stop)                                                                                                        |
#               +--------------------------------------------------------------------------------------------------------------------+
#                      receive command: 0
#                      (stop)

Header header #ヘッダ
uint16 current #現在のミッション番号
uint16 total #合計のミッション番号
uint8 status #ミッションのステータス
