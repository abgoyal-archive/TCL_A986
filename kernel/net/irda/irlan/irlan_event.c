

#include <net/irda/irlan_event.h>

char *irlan_state[] = {
	"IRLAN_IDLE",
	"IRLAN_QUERY",
	"IRLAN_CONN",
	"IRLAN_INFO",
	"IRLAN_MEDIA",
	"IRLAN_OPEN",
	"IRLAN_WAIT",
	"IRLAN_ARB",
	"IRLAN_DATA",
	"IRLAN_CLOSE",
	"IRLAN_SYNC",
};

void irlan_next_client_state(struct irlan_cb *self, IRLAN_STATE state)
{
	IRDA_DEBUG(2, "%s(), %s\n", __func__ , irlan_state[state]);

	IRDA_ASSERT(self != NULL, return;);
	IRDA_ASSERT(self->magic == IRLAN_MAGIC, return;);

	self->client.state = state;
}

void irlan_next_provider_state(struct irlan_cb *self, IRLAN_STATE state)
{
	IRDA_DEBUG(2, "%s(), %s\n", __func__ , irlan_state[state]);

	IRDA_ASSERT(self != NULL, return;);
	IRDA_ASSERT(self->magic == IRLAN_MAGIC, return;);

	self->provider.state = state;
}

