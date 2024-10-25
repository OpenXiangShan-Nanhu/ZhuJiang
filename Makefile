

idea:
	mill -i mill.idea.GenIdea/idea

init:
	git submodule update --init
	cd rocket-chip && git submodule update --init cde hardfloat

comp:
	mill -i zhujiang.compile
	mill -i zhujiang.test.compile

RTL_AGRS = --full-stacktrace --target systemverilog --split-verilog
RTL_DIR = build/rtl

ifdef PREFIX
RTL_AGRS += --prefix $(PREFIX)
endif

build-dir:
	@mkdir -p $(RTL_DIR)

tfs-top: build-dir
	mill -i zhujiang.test.runMain xijiang.TrafficSimTopMain $(RTL_AGRS) -td $(RTL_DIR)

verilog: build-dir
	mill -i zhujiang.test.runMain zhujiang.ZhujiangTop $(RTL_AGRS) -td $(RTL_DIR)

clean:
	rm -r build/*

UNAME := AxiBridge
ut-top:
	mill -i zhujiang.test.runMain zhujiang.$(UNAME)Top $(RTL_AGRS) -td build/$(UNAME)