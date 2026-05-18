/*
    hook.h: reader availability hooks

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#ifndef __HOOK_H__
#define __HOOK_H__

void ReaderHookNotifyState(const char *reader_name, const char *state,
	const char *hook_script_path);

#endif
