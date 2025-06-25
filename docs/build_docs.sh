cd .. 
pip install -e inno_control/
cd docs
rm -rf docs/*
sphinx-build -M html . ./build -v -a -E
python3 -m http.server --directory ./build/html 8000
