idea:
	mill -i mill.idea.GenIdea/idea

init:
	git submodule update --init
	cd rocket-chip/dependencies && git submodule update --init cde hardfloat diplomacy

init_ctags:
	cd rocket-chip/dependencies && git submodule update --init chisel
	ctags -R --fields=+nKz

comp:
	mill -i zhujiang.compile
	mill -i zhujiang.test.compile

RTL_AGRS = --full-stacktrace --target systemverilog --split-verilog
RTL_DIR = build/rtl

ifdef PREFIX
RTL_AGRS += --prefix $(PREFIX)
endif

MKDIR_CMD = mkdir -p $(RTL_DIR)
ifeq ($(OS),Windows_NT)
MKDIR_CMD = powershell 'New-Item -Force -Path $(RTL_DIR) -ItemType Directory | Out-Null'
endif

build-dir:
	@$(MKDIR_CMD)

tfs-top: build-dir
	mill -i zhujiang.test.runMain xijiang.TrafficSimTopMain $(RTL_AGRS) -td $(RTL_DIR)

verilog: build-dir
	mill -i zhujiang.test.runMain zhujiang.ZhujiangTop $(RTL_AGRS) -td $(RTL_DIR)

sim-top: build-dir
	mill -i zhujiang.test.runMain zhujiang.SocSystemTop $(RTL_AGRS) -td $(RTL_DIR)

clean:
	rm -r build/*

UNAME := AxiBridge
ut-top:
	mill -i zhujiang.test.runMain zhujiang.$(UNAME)Top $(RTL_AGRS) -td build/$(UNAME)