# Machine settings (same values as xlerobot-ops/config.mk).
ORIN_USER ?= orin
ORIN_HOST ?= orin.local
ORIN_DIR  ?= xlerobot-gravity-compensation
# Orin conda env (has cv2 for the camera recorder)
ORIN_PY   ?= /home/orin/miniconda3/envs/xlerobot/bin/python
# Laptop python with `pin` (Pinocchio) for the model cross-check
PIN_PY    ?= python3
