PY := python3
FIGS := system_block_diagram power_distribution_diagram cobs_frame_diagram \
        station_dataflow_diagram field_deployment_workflow matching_network_annotated
OUT  := ../images/figures

.PHONY: all clean $(FIGS)
all: $(FIGS)

$(FIGS):
	$(PY) $@.py

clean:
	rm -f $(addprefix $(OUT)/, $(addsuffix .png, $(FIGS)))
