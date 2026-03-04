# latexmk configuration for thesis
# Use: latexmk -pdf thesis.tex

$pdf_mode = 1;                 # generate PDF via pdflatex
$pdflatex = 'pdflatex -interaction=nonstopmode -halt-on-error %O %S';
$bibtex_use = 2;               # run biber when needed
$clean_ext = 'bbl run.xml bcf synctex.gz';
