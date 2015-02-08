import re

from django.template.defaultfilters import stringfilter
from google.appengine.ext import webapp

register = webapp.template.create_template_register()

bug_regexp = re.compile(r"bug (?P<bug_id>\d+)")
patch_regexp = re.compile(r"patch (?P<patch_id>\d+)")

@register.filter
@stringfilter
def webkit_linkify(value):
    value = bug_regexp.sub(r'<a href="http://webkit.org/b/\g<bug_id>">bug \g<bug_id></a>', value)
    value = patch_regexp.sub(r'<a href="https://bugs.webkit.org/attachment.cgi?id=\g<patch_id>&action=prettypatch">patch \g<patch_id></a>', value)
    return value

@register.filter
@stringfilter
def webkit_bug_id(value):
    return '<a href="http://webkit.org/b/%s">%s</a>' % (value, value)

@register.filter
@stringfilter
def webkit_attachment_id(value):
    return '<a href="https://bugs.webkit.org/attachment.cgi?id=%s&action=prettypatch">%s</a>' % (value, value)

@register.filter
@stringfilter
def results_link(status_id):
    return '<a href="/results/%s">results</a>' % status_id
