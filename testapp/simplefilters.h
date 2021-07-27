#pragma once

#ifndef SIMPLEFILTERS_H_
#define SIMPLEFILTERS_H_

#include <memory>
#include <graphengine/filter.h>

std::unique_ptr<graphengine::Filter> invoke_boxblur(unsigned width, unsigned height);

std::unique_ptr<graphengine::Filter> invoke_sobel(unsigned width, unsigned height);

std::unique_ptr<graphengine::Filter> invoke_masked_merge(unsigned width, unsigned height);

std::unique_ptr<graphengine::Filter> invoke_virtual_pad(unsigned src_width, unsigned src_height, unsigned left, unsigned right, unsigned top, unsigned bottom);

std::unique_ptr<graphengine::Filter> invoke_overlay(unsigned width, unsigned height, unsigned x0, unsigned x1, unsigned y0, unsigned y1);

#endif // SIMPLEFILTERS_H_
