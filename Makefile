COMPOSE := ./docker/run.sh
SERVICE := d1_arm_bridge
ROS_SETUP := source /setup_condition.sh

.PHONY: build up down ps logs restart shell doctor help

build:            ## Build the Docker image
	$(COMPOSE) build

up:               ## Start the container (detached)
	$(COMPOSE) up -d

down:             ## Stop and remove the container
	$(COMPOSE) down

ps:               ## Show compose service status
	$(COMPOSE) ps

logs:             ## Follow container logs
	$(COMPOSE) logs -f $(SERVICE)

restart:          ## Restart the container
	$(COMPOSE) down
	$(COMPOSE) up -d

shell:            ## Open a shell inside the running container
	$(COMPOSE) exec $(SERVICE) bash -lc '$(ROS_SETUP) && exec bash'

doctor:           ## Run container preflight without starting the bridge
	$(COMPOSE) run --rm --no-deps $(SERVICE) /doctor.sh

help:             ## Show this help
	@grep -E '^[a-z_-]+:.*##' $(MAKEFILE_LIST) | awk -F ':.*## ' '{printf "  make %-12s %s\n", $$1, $$2}'
