BRANCH ?= main

update_and_build:
	@git fetch
	@if [ "`git rev-list HEAD...origin/$(BRANCH) --count`" -ne "0" ]; then \
		echo "Updates found. Pulling changes and building..."; \
		git pull; \
		docker run --rm -v ${PWD}:/docs squidfunk/mkdocs-material build; \
	else \
		echo "No updates. Skipping build."; \
	fi

