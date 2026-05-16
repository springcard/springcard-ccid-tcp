/*
    hook.c: reader availability hooks

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#include <config.h>

#ifdef HAVE_ERRNO_H
#include <errno.h>
#endif
#ifdef HAVE_FCNTL_H
#include <fcntl.h>
#endif
#ifdef HAVE_SIGNAL_H
#include <signal.h>
#endif
#ifdef HAVE_SPAWN_H
#include <spawn.h>
#endif
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif
#ifdef HAVE_STRING_H
#include <string.h>
#endif
#ifdef HAVE_SYS_TYPES_H
#include <sys/types.h>
#endif
#ifdef HAVE_SYS_WAIT_H
#include <sys/wait.h>
#endif
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#ifdef HAVE_PTHREAD
#include <pthread.h>
#endif

#include "debug.h"
#include "hook.h"

#ifndef HOOK_SCRIPT_TIMEOUT_SECONDS
#define HOOK_SCRIPT_TIMEOUT_SECONDS 5
#endif

struct reader_hook_args
{
	char *reader_name;
	char *state;
};

#if defined(HAVE_PTHREAD) && defined(HAVE_POSIX_SPAWN) && defined(HAVE_SPAWN_H) && defined(HAVE_WAITPID) && defined(HAVE_SYS_WAIT_H) && defined(HAVE_KILL) && defined(HAVE_SIGNAL_H)
#define READER_HOOK_SUPPORTED 1
#endif

#ifdef READER_HOOK_SUPPORTED
static void free_hook_args(struct reader_hook_args *args)
{
	if (args)
	{
		free(args->reader_name);
		free(args->state);
		free(args);
	}
}

static pid_t spawn_hook_script(const struct reader_hook_args *args)
{
	pid_t pid;
	char *argv[4];
	char *envp[] = {
		"PATH=/usr/sbin:/usr/bin:/sbin:/bin",
		"IFS= \t\n",
		NULL
	};
	posix_spawn_file_actions_t file_actions;
	int file_actions_ready = 0;

	argv[0] = (char *)HOOK_SCRIPT_PATH;
	argv[1] = args->reader_name;
	argv[2] = args->state;
	argv[3] = NULL;

	if (posix_spawn_file_actions_init(&file_actions) == 0)
	{
		file_actions_ready = 1;
		(void)posix_spawn_file_actions_addopen(&file_actions, STDIN_FILENO,
			"/dev/null", O_RDONLY, 0);
		(void)posix_spawn_file_actions_addopen(&file_actions, STDOUT_FILENO,
			"/dev/null", O_WRONLY, 0);
		(void)posix_spawn_file_actions_addopen(&file_actions, STDERR_FILENO,
			"/dev/null", O_WRONLY, 0);
	}

	if (posix_spawn(&pid, HOOK_SCRIPT_PATH,
		file_actions_ready ? &file_actions : NULL, NULL, argv, envp) != 0)
	{
		if (file_actions_ready)
			(void)posix_spawn_file_actions_destroy(&file_actions);
		return -1;
	}

	if (file_actions_ready)
		(void)posix_spawn_file_actions_destroy(&file_actions);

	return pid;
}

static void terminate_hook_script(pid_t pid)
{
	int status;
	int i;

	(void)kill(pid, SIGTERM);
	for (i = 0; i < 2; i++)
	{
		if (waitpid(pid, &status, WNOHANG) == pid)
			return;
		(void)sleep(1);
	}

	(void)kill(pid, SIGKILL);
	(void)waitpid(pid, &status, 0);
}

static void *reader_hook_thread(void *opaque)
{
	struct reader_hook_args *args = opaque;
	pid_t pid;
	int status;
	int elapsed;

#ifdef HAVE_ACCESS
	if (access(HOOK_SCRIPT_PATH, X_OK) != 0)
	{
		free_hook_args(args);
		return NULL;
	}
#endif

	pid = spawn_hook_script(args);
	if (pid < 0)
	{
		DEBUG_CRITICAL2("failed to spawn hook script %s", HOOK_SCRIPT_PATH);
		free_hook_args(args);
		return NULL;
	}

	for (elapsed = 0; elapsed < HOOK_SCRIPT_TIMEOUT_SECONDS; elapsed++)
	{
		if (waitpid(pid, &status, WNOHANG) == pid)
		{
			free_hook_args(args);
			return NULL;
		}
		(void)sleep(1);
	}

	DEBUG_CRITICAL3("hook script %s timed out for reader %s",
		HOOK_SCRIPT_PATH, args->reader_name);
	terminate_hook_script(pid);

	free_hook_args(args);
	return NULL;
}
#endif

void ReaderHookNotifyState(const char *reader_name, const char *state)
{
#ifdef READER_HOOK_SUPPORTED
	pthread_t thread;
	struct reader_hook_args *args;

	if ((reader_name == NULL) || (state == NULL))
		return;

	if ((strcmp(state, "available") != 0)
		&& (strcmp(state, "unavailable") != 0))
		return;

	args = calloc(1, sizeof(*args));
	if (args == NULL)
		return;

	args->reader_name = strdup(reader_name);
	args->state = strdup(state);
	if ((args->reader_name == NULL) || (args->state == NULL))
	{
		free_hook_args(args);
		return;
	}

	if (pthread_create(&thread, NULL, reader_hook_thread, args) != 0)
	{
		free_hook_args(args);
		return;
	}

	(void)pthread_detach(thread);
#else
	(void)reader_name;
	(void)state;
#endif
}
