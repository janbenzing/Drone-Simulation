/****************************************************************************
 * drivers/leds/userled_upper.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This file provides a driver for a LED input devices.
 *
 * The LEDs driver exports a standard character driver interface. By
 * convention, the LED driver is registered as an input device at
 * /dev/btnN where N uniquely identifies the driver instance.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one LED driver */

struct userled_upperhalf_s
{
  /* Saved binding to the lower half LED driver */

  FAR const struct userled_lowerhalf_s *lu_lower;

  userled_set_t lu_supported; /* The set of supported LEDs */
  userled_set_t lu_ledset;    /* Current state of LEDs */
  sem_t lu_exclsem;           /* Supports exclusive access to the device */

  /* The following is a singly linked list of open references to the
   * LED device.
   */

  FAR struct userled_open_s *lu_open;
};

/* This structure describes the state of one open LED driver instance */

struct userled_open_s
{
  /* Supports a singly linked list */

  FAR struct userled_open_s *bo_flink;

  /* The following will be true if we are closing */

  volatile bool bo_closing;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphore helpers */

static inline int userled_takesem(sem_t *sem);
#define userled_givesem(s) sem_post(s);

/* Character driver methods */

static int     userled_open(FAR struct file *filep);
static int     userled_close(FAR struct file *filep);
static ssize_t userled_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
static int     userled_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations userled_fops =
{
  userled_open,  /* open */
  userled_close, /* close */
  NULL,          /* read */
  userled_write, /* write */
  NULL,          /* seek */
  userled_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL         /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userled_takesem
 ****************************************************************************/

static inline int userled_takesem(sem_t *sem)
{
  /* Take a count from the semaphore, possibly waiting */

  if (sem_wait(sem) < 0)
    {
      /* EINTR is the only error that we expect */

      int errcode = get_errno();
      DEBUGASSERT(errcode == EINTR);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: userled_open
 ****************************************************************************/

static int userled_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct userled_upperhalf_s *priv;
  FAR struct userled_open_s *opriv;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = (FAR struct userled_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = userled_takesem(&priv->lu_exclsem);
  if (ret < 0)
    {
      lcderr("ERROR: userled_takesem failed: %d\n", ret);
      return ret;
    }

  /* Allocate a new open structure */

  opriv = (FAR struct userled_open_s *)kmm_zalloc(sizeof(struct userled_open_s));
  if (!opriv)
    {
      lcderr("ERROR: Failled to allocate open structure\n");
      ret = -ENOMEM;
      goto errout_with_sem;
    }

  /* Attach the open structure to the device */

  opriv->bo_flink = priv->lu_open;
  priv->lu_open = opriv;

  /* Attach the open structure to the file structure */

  filep->f_priv = (FAR void *)opriv;
  ret = OK;

errout_with_sem:
  userled_givesem(&priv->lu_exclsem);
  return ret;
}

/****************************************************************************
 * Name: userled_close
 ****************************************************************************/

static int userled_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct userled_upperhalf_s *priv;
  FAR struct userled_open_s *opriv;
  FAR struct userled_open_s *curr;
  FAR struct userled_open_s *prev;
  irqstate_t flags;
  bool closing;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct userled_upperhalf_s *)inode->i_private;

  /* Handle an improbable race conditions with the following atomic test
   * and set.
   *
   * This is actually a pretty feeble attempt to handle this.  The
   * improbable race condition occurs if two different threads try to
   * close the LED driver at the same time.  The rule:  don't do
   * that!  It is feeble because we do not really enforce stale pointer
   * detection anyway.
   */

  flags = enter_critical_section();
  closing = opriv->bo_closing;
  opriv->bo_closing = true;
  leave_critical_section(flags);

  if (closing)
    {
      /* Another thread is doing the close */

      return OK;
    }

  /* Get exclusive access to the driver structure */

  ret = userled_takesem(&priv->lu_exclsem);
  if (ret < 0)
    {
      lcderr("ERROR: userled_takesem failed: %d\n", ret);
      return ret;
    }

  /* Find the open structure in the list of open structures for the device */

  for (prev = NULL, curr = priv->lu_open;
       curr && curr != opriv;
       prev = curr, curr = curr->bo_flink);

  DEBUGASSERT(curr);
  if (!curr)
    {
      lcderr("ERROR: Failed to find open entry\n");
      ret = -ENOENT;
      goto errout_with_exclsem;
    }

  /* Remove the structure from the device */

  if (prev)
    {
      prev->bo_flink = opriv->bo_flink;
    }
  else
    {
      priv->lu_open = opriv->bo_flink;
    }

  /* And free the open structure */

  kmm_free(opriv);
  ret = OK;

errout_with_exclsem:
  userled_givesem(&priv->lu_exclsem);
  return ret;
}

/****************************************************************************
 * Name: userled_write
 ****************************************************************************/

static ssize_t userled_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  FAR struct inode *inode;
  FAR struct userled_upperhalf_s *priv;
  FAR const struct userled_lowerhalf_s *lower;
  userled_set_t ledset;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct userled_upperhalf_s *)inode->i_private;

  /* Make sure that the buffer is sufficiently large to hold at least one
   * complete sample.
   *
   * REVISIT:  Should also check buffer alignment.
   */

  if (len < sizeof(userled_set_t))
    {
      lcderr("ERROR: buffer too small: %lu\n", (unsigned long)len);
      return -EINVAL;
    }

  /* Get the LED set to write.
   * REVISIT:  if sizeof(userled_set_t) > 1, then we will have to address
   * some buffer alignment issues.
   */

  DEBUGASSERT(buffer != NULL);
  ledset = *(userled_set_t *)buffer;

  /* Get exclusive access to the driver structure */

  ret = userled_takesem(&priv->lu_exclsem);
  if (ret < 0)
    {
      lcderr("ERROR: userled_takesem failed: %d\n", ret);
      return ret;
    }

  /* Read and return the current state of the LEDs */

  lower = priv->lu_lower;
  DEBUGASSERT(lower && lower->ll_ledset);
  lower->ll_ledset(lower, ledset);

  userled_givesem(&priv->lu_exclsem);
  return (ssize_t)sizeof(userled_set_t);
}

/****************************************************************************
 * Name: userled_ioctl
 ****************************************************************************/

static int userled_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct userled_upperhalf_s *priv;
  FAR const struct userled_lowerhalf_s *lower;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_priv != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct userled_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = userled_takesem(&priv->lu_exclsem);
  if (ret < 0)
    {
      lcderr("ERROR: userled_takesem failed: %d\n", ret);
      return ret;
    }

  /* Handle the ioctl command */

  ret = -EINVAL;
  switch (cmd)
    {
    /* Command:     ULEDIOC_SUPPORTED
     * Description: Report the set of LEDs supported by the hardware;
     * Argument:    A pointer to writeable userled_set_t value in which to
     *              return the set of supported LEDs.
     * Return:      Zero (OK) on success.  Minus one will be returned on failure
     *              with the errno value set appropriately.
     */

    case ULEDIOC_SUPPORTED:
      {
        FAR userled_set_t *supported = (FAR userled_set_t *)((uintptr_t)arg);

        /* Verify that a non-NULL pointer was provided */

        if (supported)
          {
            *supported = priv->lu_supported;
            ret = OK;
          }
      }
      break;

    /* Command:     ULEDIOC_SETLED
     * Description: Set the state of one LED.
     * Argument:    A read-only pointer to an instance of struct userled_s
     * Return:      Zero (OK) on success.  Minus one will be returned on failure
     *              with the errno value set appropriately.
     */

    case ULEDIOC_SETLED:
      {
        FAR struct userled_s *userled = (FAR struct userled_s *)((uintptr_t)arg);
        int led;
        bool ledon;

        /* Verify that a non-NULL pointer was provided */

        if (userled)
          {
            led   = userled->ul_led;
            ledon = userled->ul_on;

            /* Check that a valid LED is being set */

            if ((size_t)led < 8 * sizeof(userled_set_t) &&
                (priv->lu_supported & (1 << led)) != 0)
              {
                /* Update the LED state */

                if (ledon)
                  {
                    priv->lu_ledset |= (1 << led);
                  }
                else
                  {
                    priv->lu_ledset &= ~(1 << led);
                  }

                /* Set the LED state */

                lower = priv->lu_lower;
                DEBUGASSERT(lower != NULL && lower->ll_led != NULL);
                lower->ll_led(lower, led, ledon);
                ret = OK;
              }
          }
      }
      break;

    /* Command:     ULEDIOC_SETALL
     * Description: Set the state of all LEDs.
     * Argument:    A value of type userled_set_t cast to unsigned long
     * Return:      Zero (OK) on success.  Minus one will be returned on failure
     *              with the errno value set appropriately.
     */

    case ULEDIOC_SETALL:
      {
        userled_set_t ledset = (userled_set_t)((uintptr_t)arg);

        /* Verify that a valid LED set was provided */

        if ((ledset & priv->lu_supported) == ledset)
          {
            /* Update the LED state */

            priv->lu_ledset = ledset;

            /* Set the new LED state */

            lower = priv->lu_lower;
            DEBUGASSERT(lower != NULL && lower->ll_led != NULL);
            lower->ll_ledset(lower, ledset);
            ret = OK;
          }
      }
      break;

    /* Command:     ULEDIOC_GETALL
     * Description: Get the state of one LED.
     * Argument:    A write-able pointer to a userled_set_t memory location in
     *              which to return the LED state.
     * Return:      Zero (OK) on success.  Minus one will be returned on failure
     *              with the errno value set appropriately.
     */

    case ULEDIOC_GETALL:
      {
        FAR userled_set_t *ledset = (FAR userled_set_t *)((uintptr_t)arg);

        /* Verify that a non-NULL pointer was provided */

        if (ledset)
          {
            *ledset = priv->lu_ledset;
            ret = OK;
          }
      }
      break;

    default:
      lcderr("ERROR: Unrecognized command: %ld\n", cmd);
      ret = -ENOTTY;
      break;
    }

  userled_givesem(&priv->lu_exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userled_register
 *
 * Description:
 *   Bind the lower half LED driver to an instance of the upper half
 *   LED driver and register the composite character driver as the
 *   specified device.
 *
 * Input Parameters:
 *   devname - The name of the LED device to be registered.
 *     This should be a string of the form "/dev/ledN" where N is the the
 *     minor device number.
 *   lower - An instance of the platform-specific LED lower half driver.
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int userled_register(FAR const char *devname,
                     FAR const struct userled_lowerhalf_s *lower)
{
  FAR struct userled_upperhalf_s *priv;
  int ret;

  DEBUGASSERT(devname && lower);

  /* Allocate a new LED driver instance */

  priv = (FAR struct userled_upperhalf_s *)
    kmm_zalloc(sizeof(struct userled_upperhalf_s));

  if (!priv)
    {
      lcderr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Initialize the new LED driver instance */

  priv->lu_lower = lower;
  sem_init(&priv->lu_exclsem, 0, 1);

  DEBUGASSERT(lower && lower->ll_supported);
  priv->lu_supported = lower->ll_supported(lower);

  DEBUGASSERT(lower && lower->ll_ledset);
  priv->lu_ledset = 0;
  lower->ll_ledset(lower, priv->lu_ledset);

  /* And register the LED driver */

  ret = register_driver(devname, &userled_fops, 0666, priv);
  if (ret < 0)
    {
      lcderr("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_priv;
    }

  return OK;

errout_with_priv:
  sem_destroy(&priv->lu_exclsem);
  kmm_free(priv);
  return ret;
}
