BRANCH ?= main

update_and_build:
	@git fetch
	@if [ "`git rev-list HEAD...origin/$(BRANCH) --count`" -ne "0" ]; then \
		echo "Updates found. Pulling changes and building..."; \
		git pull; \
		echo "Running Docker command..."; \
		docker run --rm --tty=false -v ${PWD}:/docs squidfunk/mkdocs-material build || echo "Docker failed with exit code $$?"; \
	else \
		echo "No updates. Skipping build."; \
	fi
