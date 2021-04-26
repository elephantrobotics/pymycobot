PY2 = python2
PY = python3

.PHONY: lint
lint:
	@if [ ! -f flake8 ]; then $(PY) -m pip install flake8; fi
	flake8

.PHONY: test
test:
	@if [ ! -f pytest ]; then $(PY) -m pip install pytest; fi
	pytest -s ./tests/test_api.py

.PHONY: del
del: clean
	@if [ -d ./dist ]; then rm -r ./dist/; fi
	@if [ -d ./build ]; then rm -r ./build; fi
	@if [ -d ./pymycobot.egg-info ]; then rm -r ./pymycobot.egg-info; fi

.PHONY: install
install: del
	$(PY) setup.py install

.PHONY: release
release: del
	$(PY) setup.py sdist bdist_wheel
	twine upload dist/*

.PHONY: clean
clean:
	find . -type f -name *.pyc -delete
	find . -type d -name __pycache__ -delete
