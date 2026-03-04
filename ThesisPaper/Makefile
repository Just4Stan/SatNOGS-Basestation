# Thesis build targets
# Supports: tectonic (default), latexmk, or raw pdflatex

MAIN   = thesis
SRC    = $(MAIN).tex
PDF    = $(MAIN).pdf
BIB    = bibliography.bib
CHAPS  = $(wildcard chapters/*.tex)

# Auto-detect build tool
ifeq ($(shell command -v tectonic 2>/dev/null),)
  ifeq ($(shell command -v latexmk 2>/dev/null),)
    ENGINE = pdflatex
  else
    ENGINE = latexmk
  endif
else
  ENGINE = tectonic
endif

.PHONY: all clean watch help

all: $(PDF)  ## Build the thesis PDF

$(PDF): $(SRC) $(BIB) $(CHAPS)
ifeq ($(ENGINE),tectonic)
	tectonic $(SRC)
else ifeq ($(ENGINE),latexmk)
	latexmk -pdf $(SRC)
else
	pdflatex -interaction=nonstopmode $(SRC)
	bibtex $(MAIN)
	pdflatex -interaction=nonstopmode $(SRC)
	pdflatex -interaction=nonstopmode $(SRC)
endif

watch:  ## Rebuild on file changes (tectonic or latexmk)
ifeq ($(ENGINE),tectonic)
	tectonic -X watch $(SRC)
else ifeq ($(ENGINE),latexmk)
	latexmk -pdf -pvc $(SRC)
else
	@echo "Install tectonic or latexmk for watch mode"
endif

clean:  ## Remove build artifacts
	rm -f $(MAIN).{aux,bbl,bcf,blg,fdb_latexmk,fls,lof,log,lot,out,run.xml,synctex.gz,toc,xdv,pdf}
	rm -f chapters/*.aux

help:  ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*##' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*## "}; {printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'
