include config.mk

# Laptop drives everything; Orin steps run over ssh. On the Orin itself, run the scripts in scripts/ directly.
SSH  := ssh -o ConnectTimeout=10 $(ORIN_USER)@$(ORIN_HOST)
SSHT := ssh -t -o ConnectTimeout=10 $(ORIN_USER)@$(ORIN_HOST)
GC   := cd $(ORIN_DIR) && sg dialout -c
BIN  := target/release/gravcomp
ARM  ?= both
SCALE ?= 1.0
# Board hold feel: spring scale (1 = config gains) and at-rest friction push toward the goal (0..1).
STIFF ?= 0.5
ASSIST ?= 0.2
DEMO_SESSION := gravcomp-demo
STAMP := $(shell date +%Y%m%d-%H%M%S)

.PHONY: help test check-model sync rust-orin build probe pose rest imp board board-demo open run demo record hold release identify free-ports fetch-demo

help:
	@echo "xlerobot-gravity-compensation"
	@echo "  make test            Unit tests (laptop)"
	@echo "  make check-model     Rust gravity model vs Pinocchio (laptop, needs: pip install pin)"
	@echo "  make sync            Copy repo to the Orin (~/$(ORIN_DIR))"
	@echo "  make rust-orin       One-time: user-level rustup on the Orin"
	@echo "  make build           sync + cargo build --release on the Orin"
	@echo "  make free-ports      Stop xlerobot-ops host/bridge (they hold the serial ports)"
	@echo "  make probe           Read-only check: angles, model torque, load (moves nothing)"
	@echo "  make pose            Move ARM=$(ARM) to the 'extended' pose and hold"
	@echo "  make identify ARM=left JOINT=shoulder_lift   Break-away PWM test for one joint"
	@echo "  make run [ARM=both SCALE=0.5 JOINTS=elbow_flex,...]   Gravity compensation (Ctrl+C = hold)"
	@echo "  make imp [ARM=left SCALE=1.0 HOLD=1 GRASP=1 POSE=extended DURATION=90]   + impedance goals over UDP (tools/send_goal.py)"
	@echo "  make open ARM=left   Open the gripper (let go of the ball), gripper torque off"
	@echo "  make board [STIFF=0.5 ASSIST=0.2 DURATION=120]   Both arms grasp a board edge, hold it level (tools/level_board.py)"
	@echo "  make board-demo [NOGRASP=1]   Board hold + 3-camera recording with live tilt overlay (tmux)"
	@echo "  make demo [SCALE=..] Pose + grasp ping pong balls + gravcomp + 3-camera recording (tmux)"
	@echo "  make rest            Fold ARM=$(ARM) to the rest pose, then torque off (keeps servos cool)"
	@echo "  make hold | make release   Position-hold all arm motors | torque off (arm falls)"
	@echo "  make fetch-demo      Copy recorded demos from the Orin to ./demo"

test:
	cargo test --release

check-model:
	cargo build --release
	$(PIN_PY) tools/check_gravity_pinocchio.py

sync:
	rsync -az --delete --exclude target --exclude .git --exclude demo --exclude logs ./ $(ORIN_USER)@$(ORIN_HOST):$(ORIN_DIR)/

rust-orin:
	$(SSH) 'command -v ~/.cargo/bin/cargo >/dev/null || curl --proto =https --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal'

build: sync
	$(SSH) 'cd $(ORIN_DIR) && ~/.cargo/bin/cargo build --release 2>&1 | tail -3'

free-ports:
	-$(SSH) "pkill -f '[x]lerobot_web_bridge'; pkill -f '[x]lerobot_host'; true"

probe:
	$(SSH) "$(GC) '$(BIN) probe --arm $(ARM)'"

pose:
	$(SSHT) "$(GC) '$(BIN) pose --arm $(ARM) --name extended'"

identify:
	$(SSHT) "$(GC) '$(BIN) identify --arm $(ARM) --joint $(JOINT)'"

# Impedance: gravity comp + goal spring. HOLD=1 starts as a soft spring at the start pose. Goals: tools/send_goal.py
imp:
	$(SSHT) "mkdir -p $(ORIN_DIR)/logs && $(GC) '$(BIN) run --arm $(ARM) --scale $(SCALE) -v --listen $(if $(HOLD),--hold) $(if $(GRASP),--grasp --object \"ping pong ball\") $(if $(POSE),--pose $(POSE)) $(if $(DURATION),--duration $(DURATION)) --log logs/$(STAMP)-imp-{arm}.csv'"

# Two-arm board hold: both arms extend, grasp the board edge (Enter per arm), hold position + orientation.
# Then: python tools/level_board.py --apply   (and --watch). Ctrl+C holds; then make rest ARM=both.
board:
	$(SSHT) "mkdir -p $(ORIN_DIR)/logs && $(GC) '$(BIN) run --arm both --scale $(SCALE) -v --listen --hold --level --stiffness $(STIFF) --assist $(ASSIST) $(if $(NOGRASP),,--grasp --object \"board edge\") --pose extended $(if $(DURATION),--duration $(DURATION)) --log logs/$(STAMP)-board-{arm}.csv'"

# Board demo: window 1 records 3 cameras with a live tilt overlay; window 2 runs the board hold.
# Ctrl+C in window 2 (arms hold), then in window 1. Videos: demo/<stamp>-board/ on the Orin (make fetch-demo).
board-demo: free-ports
	-tmux kill-session -t $(DEMO_SESSION) 2>/dev/null
	tmux new-session -d -s $(DEMO_SESSION) -n record "$(SSHT) 'cd $(ORIN_DIR) && $(ORIN_PY) tools/record_cameras.py --out demo/$(STAMP)-board --overlay-board --raw --title \"XLeRobot - two-arm compliant board hold (gravity compensated)\"'; echo 'recorder exited - Enter'; read -r _"
	tmux new-window -t $(DEMO_SESSION) -n board "$(SSHT) \"mkdir -p $(ORIN_DIR)/demo/$(STAMP)-board && $(GC) '$(BIN) run --arm both --scale $(SCALE) -v --listen --hold --level --stiffness $(STIFF) --assist $(ASSIST) $(if $(NOGRASP),,--grasp --object \\\"board edge\\\") --pose extended $(if $(DURATION),--duration $(DURATION)) --log demo/$(STAMP)-board/{arm}.csv'\"; echo 'board exited - Enter'; read -r _"
	tmux new-window -t $(DEMO_SESSION) -n level "python3 tools/level_board.py --wait 300 --apply --watch; echo 'level exited - Enter'; read -r _"
	tmux select-window -t $(DEMO_SESSION):board
	tmux attach -t $(DEMO_SESSION)

# Open the gripper (let go of the ball) and turn its torque off.
open:
	$(SSHT) "$(GC) '$(BIN) gripper --arm $(ARM) --pct 60 --release'"

run:
	$(SSHT) "mkdir -p $(ORIN_DIR)/logs && $(GC) '$(BIN) run --arm $(ARM) --scale $(SCALE) -v $(if $(JOINTS),--joints $(JOINTS)) $(if $(POSE),--pose $(POSE)) --log logs/$(STAMP)-{arm}.csv'"

rest:
	$(SSH) "$(GC) '$(BIN) pose --arm $(ARM) --name rest && $(BIN) release --arm $(ARM) --yes'"

hold:
	$(SSH) "$(GC) '$(BIN) hold --arm $(ARM)'"

release:
	$(SSHT) "$(GC) '$(BIN) release --arm $(ARM) --yes'"

# Demo: window 1 records the 3 cameras; window 2 poses, grasps the balls, runs gravcomp. Ctrl+C in window 2, then window 1.
demo: free-ports
	-tmux kill-session -t $(DEMO_SESSION) 2>/dev/null
	tmux new-session -d -s $(DEMO_SESSION) -n record "$(SSHT) 'cd $(ORIN_DIR) && $(ORIN_PY) tools/record_cameras.py --out demo/$(STAMP)'; echo 'recorder exited - Enter'; read -r _"
	tmux new-window -t $(DEMO_SESSION) -n gravcomp "$(SSHT) \"mkdir -p $(ORIN_DIR)/demo/$(STAMP) && $(GC) '$(BIN) run --arm both --scale $(SCALE) --pose extended --grasp -v --log demo/$(STAMP)/{arm}.csv'\"; echo 'gravcomp exited - Enter'; read -r _"
	tmux attach -t $(DEMO_SESSION)

record:
	$(SSHT) 'cd $(ORIN_DIR) && $(ORIN_PY) tools/record_cameras.py --out demo/$(STAMP)'

fetch-demo:
	rsync -av $(ORIN_USER)@$(ORIN_HOST):$(ORIN_DIR)/demo/ ./demo/
